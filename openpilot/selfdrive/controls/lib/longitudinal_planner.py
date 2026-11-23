#!/usr/bin/env python3
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Radar-based safety decel (G80 curb/stopping improvements):
#  - close in-lane objects (curbs, parked cars, shadows the vision model misses)
#  - stopped/near-stopped leads at distance that the MPC/e2e ramp undershoots
CLOSE_OBJECT_DIST = 5.0        # m: trigger distance for the close-object decel
CLOSE_OBJECT_LANE_HALF = 1.8   # m: in-lane y threshold (|yRel|)
CLOSE_OBJECT_MARGIN = 0.5      # m: safety margin kept before a close object
CLOSE_OBJECT_MAX_DECEL = 3.0   # m/s^2: max decel from the close-object boost
STOPPED_LEAD_DIST = 80.0       # m: max distance for the stopped-lead decel
STOPPED_LEAD_SPEED = 1.0       # m/s: lead speed threshold for "stopped"
STOPPED_LEAD_MARGIN = 1.0      # m: safety margin kept before a stopped lead
STOPPED_LEAD_MAX_DECEL = 3.5   # m/s^2: max decel from the stopped-lead boost (matches ACCEL_MIN)


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt):
  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # Radar-based safety decel: boost braking for very close in-lane objects
    # (curbs, parked cars, shadows the vision model misses) and for stopped
    # leads at distance that the MPC/e2e decel ramp undershoots.
    a_radar_safety = self._radar_safety_decel(sm, v_ego)
    if a_radar_safety < output_a_target:
      output_a_target = a_radar_safety
      if a_radar_safety < -0.5:
        self.output_should_stop = True

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def _radar_safety_decel(self, sm, v_ego):
    # Returns a non-positive accel to apply on top of the planned accel.
    # 0.0 when no boost is needed. Conservative: only ever increases braking.
    a_safety = 0.0
    if v_ego < 1.0:
      return a_safety

    # Close in-lane objects from raw radar points (curbs, parked cars, shadows)
    try:
      points = sm['radarTracks'].points
    except Exception:
      points = []
    for p in points:
      d, y = p.dRel, p.yRel
      # skip objects moving away from us faster than 0.5 m/s relative
      if d < 0.5 or d > CLOSE_OBJECT_DIST or abs(y) > CLOSE_OBJECT_LANE_HALF or p.vRel > 0.5:
        continue
      stop_dist = max(d - CLOSE_OBJECT_MARGIN, 0.3)
      a_req = v_ego ** 2 / (2.0 * stop_dist)
      a_safety = min(a_safety, -min(a_req, CLOSE_OBJECT_MAX_DECEL))

    # Stopped/near-stopped leads at distance (MPC/e2e ramp can undershoot)
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      if not lead.present:
        continue
      d, y = lead.dRel, lead.yRel
      if d < 0.5 or d > STOPPED_LEAD_DIST or abs(y) > CLOSE_OBJECT_LANE_HALF:
        continue
      if lead.vLead > STOPPED_LEAD_SPEED:
        continue
      stop_dist = max(d - STOPPED_LEAD_MARGIN, 0.5)
      a_req = v_ego ** 2 / (2.0 * stop_dist)
      a_safety = min(a_safety, -min(a_req, STOPPED_LEAD_MAX_DECEL))

    return a_safety

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
