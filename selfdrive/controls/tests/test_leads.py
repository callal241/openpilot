from types import SimpleNamespace

import cereal.messaging as messaging

from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, match_vision_to_track, use_lead_lateral_sanity
from openpilot.selfdrive.test.process_replay import replay_process_with_name


class TestLeads:
  def test_g80_mando_radar_uses_lateral_sanity_check(self):
    g80 = SimpleNamespace(brand="hyundai", carFingerprint="GENESIS_G80", flags=HyundaiFlags.MANDO_RADAR)
    palisade = SimpleNamespace(brand="hyundai", carFingerprint="HYUNDAI_PALISADE", flags=HyundaiFlags.MANDO_RADAR)
    g80_no_radar = SimpleNamespace(brand="hyundai", carFingerprint="GENESIS_G80", flags=0)

    assert use_lead_lateral_sanity(g80)
    assert not use_lead_lateral_sanity(palisade)
    assert not use_lead_lateral_sanity(g80_no_radar)

  def test_radar_match_rejects_wrong_lane_track_with_lateral_sanity_check(self):
    lead = SimpleNamespace(x=[50.0], y=[0.0], v=[10.0], xStd=[1.0], yStd=[0.3], vStd=[1.0])
    wrong_lane_track = SimpleNamespace(dRel=lead.x[0] - RADAR_TO_CAMERA, yRel=4.0, vRel=0.0)

    assert match_vision_to_track(10.0, lead, {1: wrong_lane_track}, check_lateral=True) is None

  def test_radar_match_keeps_upstream_behavior_without_lateral_sanity_check(self):
    lead = SimpleNamespace(x=[50.0], y=[0.0], v=[10.0], xStd=[1.0], yStd=[0.3], vStd=[1.0])
    wrong_lane_track = SimpleNamespace(dRel=lead.x[0] - RADAR_TO_CAMERA, yRel=4.0, vRel=0.0)

    assert match_vision_to_track(10.0, lead, {1: wrong_lane_track}) is wrong_lane_track

  def test_radar_match_accepts_laterally_consistent_track(self):
    lead = SimpleNamespace(x=[50.0], y=[0.0], v=[10.0], xStd=[1.0], yStd=[0.3], vStd=[1.0])
    same_lane_track = SimpleNamespace(dRel=lead.x[0] - RADAR_TO_CAMERA, yRel=0.4, vRel=0.0)

    assert match_vision_to_track(10.0, lead, {1: same_lane_track}, check_lateral=True) is same_lane_track

  def test_radar_fault(self):
    # if there's no radar-related can traffic, radard should either not respond or respond with an error
    # this is tightly coupled with underlying car radar_interface implementation, but it's a good sanity check
    def single_iter_pkg():
      # single iter package, with meaningless cans and empty carState/modelV2
      msgs = []
      for _ in range(500):
        can = messaging.new_message("can", 1)
        cs = messaging.new_message("carState")
        cp = messaging.new_message("carParams")
        msgs.append(can.as_reader())
        msgs.append(cs.as_reader())
        msgs.append(cp.as_reader())
      model = messaging.new_message("modelV2")
      msgs.append(model.as_reader())

      return msgs

    msgs = [m for _ in range(3) for m in single_iter_pkg()]
    out = replay_process_with_name("card", msgs, fingerprint=TOYOTA.TOYOTA_COROLLA_TSS2)
    states = [m for m in out if m.which() == "liveTracks"]
    failures = [not state.valid for state in states]

    assert len(states) == 0 or all(failures)
