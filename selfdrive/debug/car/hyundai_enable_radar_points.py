#!/usr/bin/env python3
"""Some Hyundai radars can be reconfigured to output (debug) radar points on bus 1.
Reconfiguration is done over UDS by reading/writing to 0x0142 using the Read/Write Data By Identifier
endpoints (0x22 & 0x2E). This script checks your radar firmware version against a list of known
firmware versions. If you want to try on a new radar make sure to note the default config value
in case it's different from the other radars and you need to revert the changes.

After changing the config the car should not show any faults when openpilot is not running.
These config changes are persistent across car reboots. You need to run this script again
to go back to the default values.

USE AT YOUR OWN RISK! Safety features, like AEB and FCW, might be affected by these changes."""

import sys
import argparse
from typing import NamedTuple
from subprocess import check_output, CalledProcessError

from opendbc.car.carlog import carlog
from opendbc.car.uds import UdsClient, SESSION_TYPE, DATA_IDENTIFIER_TYPE, MessageTimeoutError, NegativeResponseError
from opendbc.car.structs import CarParams
from panda.python import Panda

class ConfigValues(NamedTuple):
  default_config: bytes
  tracks_enabled: bytes
  diagnostic_session: SESSION_TYPE | int = 0x07
  write_supported: bool = True
  note: str = ""


class RadarProbe(NamedTuple):
  bus: int
  session_type: SESSION_TYPE | int
  uds_client: UdsClient
  fw_version: bytes
  current_config: bytes


FW_VERSION_DATA_ID: DATA_IDENTIFIER_TYPE = 0xf100
CONFIG_DATA_ID: DATA_IDENTIFIER_TYPE = 0x0142
DEVELOPER_DIAGNOSTIC_SESSION = 0x07

# If your radar supports changing data identifier 0x0142 as well make a PR to
# this file to add your firmware version. Make sure to post a drive as proof!
# NOTE: these firmware versions do not match what openpilot uses
#       because this script uses a different diagnostic session type
SUPPORTED_FW_VERSIONS = {
  # 2020 SONATA
  b"DN8_ SCC FHCUP      1.00 1.00 99110-L0000\x19\x08)\x15T    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  b"DN8_ SCC F-CUP      1.00 1.00 99110-L0000\x19\x08)\x15T    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2021 SONATA HYBRID
  b"DNhe SCC FHCUP      1.00 1.00 99110-L5000\x19\x04&\x13'    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  b"DNhe SCC FHCUP      1.00 1.02 99110-L5000 \x01#\x15#    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2020 PALISADE
  b"LX2_ SCC FHCUP      1.00 1.04 99110-S8100\x19\x05\x02\x16V    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2022 PALISADE
  b"LX2_ SCC FHCUP      1.00 1.00 99110-S8110!\x04\x05\x17\x01    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2020 SANTA FE
  b"TM__ SCC F-CUP      1.00 1.03 99110-S2000\x19\x050\x13'    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2020 GENESIS G70
  b'IK__ SCC F-CUP      1.00 1.02 96400-G9100\x18\x07\x06\x17\x12    ': ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2018 GENESIS G80 / DH SCC radar.
  # This radar rejects the developer session used by older Mando radars, but it
  # exposes DID 0x0142 over extended diagnostics. With a direct radar harness,
  # this observed config already outputs radar tracks, so do not write to it.
  b"DH__ SCC FHCUP      1.00 1.01 96400-B1120         ": ConfigValues(
    default_config=b"\x01\x02\x00\x01\x01",
    tracks_enabled=b"\x01\x02\x00\x01\x01",
    diagnostic_session=SESSION_TYPE.EXTENDED_DIAGNOSTIC,
    write_supported=False,
    note="Genesis G80 DH radar: verified direct-harness config, no UDS write needed"),
  # 2019 SANTA FE
  b"TM__ SCC F-CUP      1.00 1.00 99110-S1210\x19\x01%\x168    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  b"TM__ SCC F-CUP      1.00 1.02 99110-S2000\x18\x07\x08\x18W    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2021 K5 HEV
  b"DLhe SCC FHCUP      1.00 1.02 99110-L7000 \x01 \x102    ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  # 2022 Niro EV
  b"DEev SCC F-CUP      1.00 1.00 99110-Q4600\x01\x42      ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
  b"DEev SCC F-CUP      1.00 1.00 99110-Q4600 \x07\x03\t%      ": ConfigValues(
    default_config=b"\x00\x00\x00\x01\x00\x00",
    tracks_enabled=b"\x00\x00\x00\x01\x00\x01"),
}


def session_name(session_type: SESSION_TYPE | int) -> str:
  try:
    return SESSION_TYPE(session_type).name
  except ValueError:
    return f"0x{session_type:02x}"


def try_probe_radar(panda: Panda, bus: int, session_type: SESSION_TYPE | int) -> tuple[RadarProbe | None, Exception | None]:
  uds_client = UdsClient(panda, 0x7D0, bus=bus)
  try:
    uds_client.diagnostic_session_control(session_type)
    fw_version = uds_client.read_data_by_identifier(FW_VERSION_DATA_ID)
    current_config = uds_client.read_data_by_identifier(CONFIG_DATA_ID)
    return RadarProbe(bus, session_type, uds_client, fw_version, current_config), None
  except (MessageTimeoutError, NegativeResponseError, ValueError) as e:
    return None, e


def find_supported_radar(panda: Panda, buses: list[int], debug: bool) -> RadarProbe:
  probe_order = [
    SESSION_TYPE.DEFAULT,
    SESSION_TYPE.EXTENDED_DIAGNOSTIC,
    DEVELOPER_DIAGNOSTIC_SESSION,
  ]
  unsupported: list[RadarProbe] = []
  failures: list[tuple[int, SESSION_TYPE | int, Exception]] = []

  for bus in buses:
    for session_type in probe_order:
      probe, error = try_probe_radar(panda, bus, session_type)
      if probe is None:
        if error is not None:
          failures.append((bus, session_type, error))
        continue

      if debug:
        print(f"[probe] bus {bus}, session {session_name(session_type)}, fw={probe.fw_version!r}, config=0x{probe.current_config.hex()}")

      if probe.fw_version in SUPPORTED_FW_VERSIONS:
        config_values = SUPPORTED_FW_VERSIONS[probe.fw_version]
        if session_type == config_values.diagnostic_session:
          return probe

        preferred_probe, preferred_error = try_probe_radar(panda, bus, config_values.diagnostic_session)
        if preferred_probe is not None:
          return preferred_probe
        if preferred_error is not None:
          failures.append((bus, config_values.diagnostic_session, preferred_error))

      unsupported.append(probe)
      break

  if unsupported:
    print("radar firmware was readable but is not supported by this script:")
    seen = set()
    for probe in unsupported:
      key = (probe.bus, probe.fw_version, probe.current_config)
      if key in seen:
        continue
      seen.add(key)
      print(f"  bus {probe.bus}: fw={probe.fw_version!r}, config=0x{probe.current_config.hex()}")
  else:
    print("no radar responded on the requested bus(es)")

  if debug and failures:
    print("\nprobe failures:")
    for bus, session_type, error in failures:
      print(f"  bus {bus}, session {session_name(session_type)}: {error}")

  print("radar not supported! (aborted)")
  sys.exit(1)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='configure radar to output points (or reset to default)')
  parser.add_argument('--default', action="store_true", default=False, help='reset to default configuration (default: false)')
  parser.add_argument('--check-only', action="store_true", default=False, help='only identify radar/configuration; never write')
  parser.add_argument('--debug', action="store_true", default=False, help='enable debug output (default: false)')
  parser.add_argument('--bus', type=int, help='can bus to use (default: auto-scan 0, 2, 1)')
  args = parser.parse_args()

  if args.debug:
    carlog.setLevel('DEBUG')

  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please kill openpilot before running this script! (aborted)")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1: # 1 == no process found (pandad not running)
      raise e

  confirm = input("power on the vehicle keeping the engine off (press start button twice) then type OK to continue: ").upper().strip()
  if confirm != "OK":
    print("\nyou didn't type 'OK! (aborted)")
    sys.exit(0)

  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  buses = [args.bus] if args.bus is not None else [0, 2, 1]
  print("\n[FIND RADAR]")
  probe = find_supported_radar(panda, buses, args.debug)
  uds_client = probe.uds_client

  print("[HARDWARE/SOFTWARE VERSION]")
  fw_version = probe.fw_version
  print(fw_version)
  print(f"bus: {probe.bus}, diagnostic session: {session_name(probe.session_type)}")

  print("[GET CONFIGURATION]")
  current_config = probe.current_config
  config_values = SUPPORTED_FW_VERSIONS[fw_version]
  new_config = config_values.default_config if args.default else config_values.tracks_enabled
  print(f"current config: 0x{current_config.hex()}")

  if config_values.note:
    print(config_values.note)

  if args.check_only:
    print("[DONE]")
    print("\ncheck-only requested; no configuration was changed")
    sys.exit(0)

  if not config_values.write_supported:
    if current_config == new_config:
      print("[DONE]")
      print("\ncurrent config is already the desired configuration; no radar write is needed")
      sys.exit(0)

    print("this radar/config is read-only in this script; refusing to write an unverified value (aborted)")
    sys.exit(1)

  if current_config != new_config:
    if not args.default and current_config != config_values.default_config:
      print("\ncurrent config does not match expected default! (aborted before writing)")
      sys.exit(1)

    print("[CHANGE CONFIGURATION]")
    print(f"new config:     0x{new_config.hex()}")
    uds_client.write_data_by_identifier(CONFIG_DATA_ID, new_config)

    verified_config = uds_client.read_data_by_identifier(CONFIG_DATA_ID)
    if verified_config != new_config:
      print(f"\nwrite verification failed: read back 0x{verified_config.hex()} (aborted)")
      sys.exit(1)

    print("[DONE]")
    print("\nrestart your vehicle and ensure there are no faults")
    if not args.default:
      print("you can run this script again with --default to go back to the original (factory) settings")
  else:
    print("[DONE]")
    print("\ncurrent config is already the desired configuration")
    sys.exit(0)
