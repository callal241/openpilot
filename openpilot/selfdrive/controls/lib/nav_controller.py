#!/usr/bin/env python3
"""
Navigation-driven control behaviors (rule-based Nav Alpha control).

The e2e model has no map input, so these rule-based overrides add
navigation awareness on top of it. Consumes the active route
(navInstruction + navRoute) and GPS (liveLocationKalman) and produces:

  1. pre-exit lane change requests (off ramp / fork) for the DesireHelper
  2. a pre-turn speed cap for the longitudinal planner
  3. GPS route curvature for turn/fork trajectory blending in controlsd

Each daemon instantiates its own controller with its own dt; the state is
small and independent per daemon. All behaviors are conservative:
the speed cap only ever lowers the target speed, the lane change request
is a hint that the DesireHelper safety gates still apply, and the
curvature blend only engages when the model is going straight while the
route diverges.
"""
import math

import numpy as np

from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.navigation.route import RouteTracker

LaneChangeState = log.LaneChangeState

# --- pre-turn deceleration ---------------------------------------------------
# Target approach speeds (m/s) at the maneuver point, by maneuver type.
TURN_TARGET_SPEEDS = {
  'slightLeft': 17.0,
  'slightRight': 17.0,
  'turnLeft': 6.0,
  'turnRight': 6.0,
  'sharpLeft': 4.5,
  'sharpRight': 4.5,
  'offRamp': 15.0,
  'fork': 12.0,
  'roundabout': 4.0,
}
NAV_DECEL_MAX = 2.2            # m/s^2, comfortable braking for the approach profile
NAV_DECEL_START_DIST = 300.0   # m, begin shaping the speed cap this far out

# --- pre-exit lane change ------------------------------------------------------
NAV_LC_MANEUVERS = ('offRamp', 'fork')
NAV_LC_START_DIST = 800.0      # m, start looking for an exit lane change
NAV_LC_MIN_DIST = 300.0        # m, give up this close (driver's turn)
NAV_LC_COOLDOWN = 5.0          # s after an attempt before another is allowed
NAV_LC_SPEED_MIN = 9.0         # m/s, ~20 mph
NAV_LC_LANE_PROB_MIN = 0.5     # target-side lane line must be visible

# --- turn/fork trajectory blend -------------------------------------------------
NAV_BLEND_MANEUVERS = ('turnLeft', 'turnRight', 'sharpLeft', 'sharpRight', 'fork', 'offRamp', 'roundabout')
NAV_BLEND_GENTLE = ('fork', 'offRamp')
NAV_BLEND_GENTLE_MAX = 0.7     # keep 30% model lane centering on gentle forks/exits
NAV_BLEND_START_DIST = 35.0    # m, begin cross-fading to the route curvature
NAV_BLEND_FULL_DIST = 10.0     # m, fully on the route curvature
NAV_ROUTE_KAPPA_MIN = 0.004    # 1/m, ignore route curves flatter than this
NAV_MODEL_KAPPA_MAX = 0.002    # 1/m, model must be going straight for the blend
NAV_GPS_STD_MAX = 3.0          # m, abort the blend on poor GPS
NAV_GPS_STALE_S = 1.0          # s, max age of the GPS fix
NAV_NAV_STALE_S = 5.0          # s, max age of navInstruction
NAV_LAT_ACCEL_MAX = 2.5        # m/s^2, cap on the blended lateral acceleration
NAV_ROUTE_KAPPA_MAX = 0.05     # 1/m, sanity cap on the route curvature
NAV_YAW_DONE_DEG = 10.0        # deg, consider the turn complete within this of the target heading
NAV_MIN_SPEED = 3.0            # m/s, no blend at crawl/standstill


class NavController:
  def __init__(self, dt: float, params: Params | None = None):
    self.dt = dt
    self.params = params if params is not None else Params()
    # get_bool does not apply the key default, so read raw: missing -> enabled (default ON)
    raw = self.params.get("NavControlEnabled", block=False)
    self.enabled = raw is None or raw == b"1"

    # outputs
    self.lane_change_request = 0        # -1 left, 0 none, 1 right
    self.v_nav_cap = None               # m/s, or None when not shaping speed
    self.blend_active = False
    self.blend_alpha = 0.0
    self.kappa_route = 0.0              # model convention: positive = right

    # state
    self._tracker: RouteTracker | None = None
    self._tracker_route_key = None
    self._kappa_filter = FirstOrderFilter(0.0, 0.3, dt)
    self._lc_cooldown = 0.0
    self._lc_in_progress = False
    self._lc_done_key = None
    self._lc_pending_key = None

  # -- route/GPS helpers -------------------------------------------------------
  def _route_active(self, sm) -> bool:
    return (self.enabled and sm.seen['navInstruction'] and sm.valid['navInstruction'] and
            (sm.frame - sm.recv_frame['navInstruction']) * self.dt < NAV_NAV_STALE_S)

  def _update_tracker(self, sm):
    coords = sm['navRoute'].coordinates
    if len(coords) < 2:
      self._tracker = None
      self._tracker_route_key = None
      return
    key = (len(coords), coords[0].latitude, coords[0].longitude, coords[-1].latitude, coords[-1].longitude)
    if key != self._tracker_route_key:
      route = np.array([[c.longitude, c.latitude] for c in coords], dtype=np.float64)
      self._tracker = RouteTracker(route)
      self._tracker_route_key = key
      self._kappa_filter.x = 0.0
    gps = sm['liveLocationKalman'].positionGeodetic
    if gps.valid and len(gps.value) >= 2:
      self._tracker.project(float(gps.value[1]), float(gps.value[0]))

  def _gps_ok(self, sm) -> tuple[bool, float]:
    """(gps fresh and accurate enough, yaw in NED radians)"""
    if not sm.seen['liveLocationKalman'] or (sm.frame - sm.recv_frame['liveLocationKalman']) * self.dt > NAV_GPS_STALE_S:
      return False, 0.0
    gps = sm['liveLocationKalman'].positionGeodetic
    if not gps.valid or len(gps.value) < 2:
      return False, 0.0
    std = gps.std[0] if len(gps.std) > 0 else 0.0
    if std > NAV_GPS_STD_MAX:
      return False, 0.0
    yaw = float(sm['liveLocationKalman'].calibratedOrientationNED.value[2])
    return True, yaw

  def _route_points_ahead(self, s0: float, s1: float) -> np.ndarray | None:
    """Interpolated route points as (east, north) meters between arc lengths s0..s1."""
    tr = self._tracker
    if tr is None or not np.isfinite(tr.progress) or s1 <= s0:
      return None
    d = tr.distance
    if d[-1] < s1:
      s1 = d[-1]
    if s1 - s0 < 5.0:
      return None
    n = 8
    s = np.linspace(s0, s1, n)
    idx = np.clip(np.searchsorted(d, s, side='right') - 1, 0, len(d) - 2)
    seg_len = np.maximum(d[idx + 1] - d[idx], 1e-6)
    frac = (s - d[idx]) / seg_len
    east = tr.path[idx, 0] + frac * (tr.path[idx + 1, 0] - tr.path[idx, 0])
    north = tr.path[idx, 1] + frac * (tr.path[idx + 1, 1] - tr.path[idx, 1])
    return np.column_stack((east, north))

  @staticmethod
  def _to_vehicle_frame(east: np.ndarray, north: np.ndarray, yaw: float) -> np.ndarray:
    """(east, north) meters around the ego -> (X forward, Y left) vehicle frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    X = north * c + east * s
    Y = north * s - east * c
    return np.column_stack((X, Y))

  def _route_kappa(self, yaw: float) -> float:
    """Curvature of the route 5-35 m ahead, model convention (positive = right)."""
    tr = self._tracker
    if tr is None or not np.isfinite(tr.progress):
      return 0.0
    pts = self._route_points_ahead(tr.progress + 5.0, tr.progress + 35.0)
    if pts is None or len(pts) < 3:
      return 0.0
    p = self._to_vehicle_frame(pts[:, 0], pts[:, 1], yaw)
    mid = len(p) // 2
    a = p[mid] - p[0]
    b = p[-1] - p[mid]
    la, lb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if la < 1e-3 or lb < 1e-3:
      return 0.0
    cross = a[0] * b[1] - a[1] * b[0]
    kappa = 2.0 * cross / (la * lb * float(np.linalg.norm(a + b)))
    # vehicle frame has Y left, so positive = left; model convention is positive = right
    return -float(kappa)

  def _route_heading(self, s: float):
    """Route heading (NED radians) at arc length s, or None."""
    tr = self._tracker
    if tr is None or not np.isfinite(tr.progress):
      return None
    d = tr.distance
    if s < d[0] or s >= d[-1]:
      return None
    i = min(int(np.searchsorted(d, s, side='right') - 1), len(d) - 2)
    e = tr.path[i + 1, 0] - tr.path[i, 0]
    n = tr.path[i + 1, 1] - tr.path[i, 1]
    if abs(e) < 1e-6 and abs(n) < 1e-6:
      return None
    return math.atan2(e, n)

  # -- main update ---------------------------------------------------------------
  def update(self, sm, lane_change_state=None, left_edge_detected=False, right_edge_detected=False):
    """
    sm: SubMaster subscribed to carState, modelV2, navInstruction, navRoute, liveLocationKalman
    lane_change_state: modeld only, the DesireHelper state to track attempt completion
    """
    self._lc_cooldown = max(0.0, self._lc_cooldown - self.dt)
    CS = sm['carState']

    route_active = self._route_active(sm)
    self._update_tracker(sm)

    # 1. pre-exit lane change request
    self.lane_change_request = 0
    progress_ok = self._tracker is not None and np.isfinite(self._tracker.progress)
    if route_active and progress_ok:
      nav = sm['navInstruction']
      mtype, mod = nav.maneuverType, nav.maneuverModifier
      dist = nav.maneuverDistance
      if mtype in NAV_LC_MANEUVERS and NAV_LC_MIN_DIST < dist < NAV_LC_START_DIST:
        side = 1 if mod in ('right', 'slightRight') else (-1 if mod in ('left', 'slightLeft') else 0)
        key = (mtype, mod, int((self._tracker.progress + dist) // 50))
        if side != 0 and not self._lc_in_progress and self._lc_cooldown <= 0.0 and key != self._lc_done_key:
          in_edge = right_edge_detected if side > 0 else left_edge_detected
          probs = sm['modelV2'].laneLineProbs
          lane_prob = probs[1] if (side > 0 and len(probs) > 1) else (probs[0] if len(probs) > 0 else 0.0)
          bsm = CS.rightBlindspot if side > 0 else CS.leftBlindspot
          if not in_edge and lane_prob > NAV_LC_LANE_PROB_MIN and not bsm and              not CS.steeringPressed and not CS.brakePressed and CS.vEgo >= NAV_LC_SPEED_MIN:
            self.lane_change_request = side
            self._lc_pending_key = key

    # track attempt completion (modeld passes the DesireHelper state)
    if lane_change_state is not None:
      if self.lane_change_request != 0 and lane_change_state == LaneChangeState.laneChangeStarting:
        self._lc_in_progress = True
        self.lane_change_request = 0
      elif lane_change_state == LaneChangeState.off:
        if self._lc_in_progress:
          self._lc_in_progress = False
          self._lc_done_key = self._lc_pending_key
          self._lc_cooldown = NAV_LC_COOLDOWN

    # 2. pre-turn speed cap
    self.v_nav_cap = None
    if route_active:
      nav = sm['navInstruction']
      v_target = TURN_TARGET_SPEEDS.get(nav.maneuverType)
      dist = nav.maneuverDistance
      if v_target is not None and dist < NAV_DECEL_START_DIST:
        # kinematic approach: the fastest speed that can still reach v_target at the
        # maneuver with NAV_DECEL_MAX braking; shrinks to v_target as dist -> 0
        self.v_nav_cap = math.sqrt(max(v_target ** 2 + 2.0 * NAV_DECEL_MAX * max(dist, 0.0), 0.0))

    # 3. turn/fork trajectory blend
    self.blend_active = False
    self.blend_alpha = 0.0
    if route_active and self._tracker is not None and CS.vEgo > NAV_MIN_SPEED and not CS.steeringPressed:
      nav = sm['navInstruction']
      mtype = nav.maneuverType
      dist = nav.maneuverDistance
      if mtype in NAV_BLEND_MANEUVERS and dist < NAV_BLEND_START_DIST:
        gps_ok, yaw = self._gps_ok(sm)
        if gps_ok:
          self.kappa_route = self._kappa_filter.update(
            float(np.clip(self._route_kappa(yaw), -NAV_ROUTE_KAPPA_MAX, NAV_ROUTE_KAPPA_MAX)))
          model_kappa = sm['modelV2'].action.desiredCurvature
          if abs(self.kappa_route) > NAV_ROUTE_KAPPA_MIN and abs(model_kappa) < NAV_MODEL_KAPPA_MAX:
            # abort once the vehicle heading aligns with the route past the maneuver
            target_heading = self._route_heading(self._tracker.progress + dist + 25.0)
            done = False
            if target_heading is not None:
              d_heading = abs(math.atan2(math.sin(yaw - target_heading), math.cos(yaw - target_heading)))
              done = d_heading < math.radians(NAV_YAW_DONE_DEG)
            if not done:
              alpha = (NAV_BLEND_START_DIST - dist) / (NAV_BLEND_START_DIST - NAV_BLEND_FULL_DIST)
              alpha = min(max(alpha, 0.0), 1.0)
              if mtype in NAV_BLEND_GENTLE:
                alpha = min(alpha, NAV_BLEND_GENTLE_MAX)
              kappa_max = NAV_LAT_ACCEL_MAX / max(CS.vEgo, 5.0) ** 2
              self.kappa_route = max(min(self.kappa_route, kappa_max), -kappa_max)
              self.blend_active = True
              self.blend_alpha = alpha
