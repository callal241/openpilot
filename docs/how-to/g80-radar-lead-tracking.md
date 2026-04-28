# Genesis G80 radar lead tracking handoff

Last updated: 2026-04-28

This note documents the G80 radar lead-switching investigation so another Codex session can continue from the same facts without rediscovering the route.

## Vehicle and symptom

- Vehicle fingerprint: `GENESIS_G80`
- Brand: `hyundai`
- Relevant car flag from the investigated route: `HyundaiFlags.MANDO_RADAR` (`CP.flags == 4096`)
- `radarUnavailable == False`
- `openpilotLongitudinalControl == True`
- Symptom: radar-backed lead selection can switch to cars in the wrong lane. The bad object is already present in parsed radar tracks, then `radard.py` can select it as the lead because distance and velocity look sane.

## Investigated drive

- comma SSH target used during the investigation: `comma@192.168.2.211`
- Local SSH key path used: `C:\Users\User\.ssh\comma_github_ed25519`
- Device route path: `/data/media/0/realdata/00000137--a984a5c1cc--0..14`
- Local copied logs: `tmp_last_drive\00000137--a984a5c1cc`
- Approximate drive time: 2026-04-28 13:14-13:27 UTC

The local logs may be untracked or ignored. If they are missing on a different computer, pull the same route from the comma before re-running analysis.

## What the logs showed

The radar is not missing the required track messages:

- Raw CAN radar slots `0x500..0x51f` are present on the radar bus throughout the route.
- Trigger message `0x51f` appeared 17,517 times over about 877 seconds, roughly 20 Hz.
- `liveTracks` published 17,516 rows.
- `liveTracks` was non-empty for 15,844 rows.
- Average parsed points when present: about 5.1.
- Max parsed points: 18.

The issue is in lead fusion/selection, not in enabling the radar or getting basic radar traffic from the radar.

Bad-match evidence from the route:

- There were 5,696 samples with both raw radar track and model lead data available.
- 973 samples would pass the old distance/velocity sanity checks but fail a lateral sanity gate.
- 25 lead switches involved those laterally bad rows.
- Example at `t=256.88s`, segment 4: raw track id `136319` had `(d,y,vRel)=(40.9,13.91,-10.7)` while model lead was `(x,y,v,prob,yStd)=(46.9,2.32,0.6,0.86,0.45)`. The lateral difference was about 16.23 m.
- Example at `t=239.53s`, segment 3: raw track id `126895` had `yRel=-8.00` with model probability `0.99`. The lateral difference was about 9.38 m.

Parsed `liveTracks` also contain wide lateral offsets, including tracks around 8 m to 24 m from the ego lane. That is useful diagnostic evidence, but the first fix should be at lead matching so raw diagnostic tracks remain visible.

## Why `enable_radar_tracks.py` is not the fix

`enable_radar_tracks.py` is for radars that need a UDS configuration write before they broadcast object track messages. This G80 route already has radar object messages and parsed `liveTracks`, so the enable script does not address the observed failure.

The standalone enable script also does not appear to include this G80 radar firmware target in its whitelist. Do not try to force that path unless a future CAN/UDS investigation proves the radar is actually configured wrong.

## Current code direction

The recommended change is a G80-specific lateral sanity check during vision-to-radar lead matching:

- Code: `selfdrive/controls/radard.py`
- Tests: `selfdrive/controls/tests/test_leads.py`
- Gate function: `use_lead_lateral_sanity(CP)`
- Intended gate: only `CP.brand == "hyundai"`, `CP.carFingerprint == "GENESIS_G80"`, and `CP.flags & HyundaiFlags.MANDO_RADAR`

The matching function keeps upstream/default behavior unless `check_lateral=True`. For the G80 Mando radar path, it compares parsed radar lateral position to the model lead lateral position:

```python
vision_y = -lead.y[0]
lat_tolerance = lead.yStd[0] * 3.0
lat_sane = abs(track.yRel - vision_y) < max(lat_tolerance, 1.5)
```

This means a radar point can still be used when it agrees with the model lead laterally, but wrong-lane points that only match in distance and velocity are rejected and the lead can fall back to vision.

## Why this is not applied to every Hyundai

Do not make the lateral gate generic without new evidence. Other supported cars, for example a Hyundai Palisade, may not show this G80-specific lateral radar behavior, and openpilot's existing matching logic historically accepts the best probability track after distance and velocity sanity checks.

A generic lateral gate can change behavior for cars that are already working, especially when the model's lateral uncertainty is high or a radar interface reports lateral position differently. Keep the current fix scoped to the failing G80 Mando radar case unless another route proves the same failure on another car.

## Flags and configuration notes

Do not add Hyundai flags just to improve control. Existing flags describe architecture or message layout; incorrect flags can break parsing or safety behavior.

From the investigated route:

- `ENHANCED_SCC` is not supported by evidence here. ESCC message `0x2ab` had count 0.
- Do not set checksum, legacy, camera SCC, CAN-FD, FCA, or LFA flags unless CAN evidence proves this vehicle needs them.
- The best current option is the lead-fusion lateral sanity gate, not more car flags.

## If lead tracking still needs adjustment

Start with `selfdrive/controls/radard.py`:

1. Keep `use_lead_lateral_sanity(CP)` scoped to proven vehicles.
2. Adjust only the lateral tolerance if same-lane G80 leads are rejected too often.
3. Compare `track.yRel` against `-lead.y[0]`; the sign matters.
4. Leave parsed `liveTracks` unfiltered unless there is a separate downstream consumer bug.
5. If parser-level filtering becomes necessary, make it G80-specific and preserve enough raw/parsed data for diagnostics.

When testing a new route, look for these measurements:

- Count raw `0x500..0x51f` radar messages on the radar bus.
- Count `liveTracks` publishes and non-empty rows.
- For every selected radar lead, log `trackId`, `dRel`, `yRel`, `vRel`, model lead `x`, `y`, `v`, `prob`, `yStd`, and the lateral difference.
- Confirm whether bad lead switches pass distance/velocity checks and fail lateral checks.

## Verification already done locally

These checks passed after adding the scoped lateral sanity behavior:

```powershell
python -m py_compile selfdrive\controls\radard.py selfdrive\controls\tests\test_leads.py
git diff --check
```

Focused AST-extracted checks also passed:

- G80 with `MANDO_RADAR` enables lateral sanity.
- Palisade with `MANDO_RADAR` does not enable lateral sanity.
- G80 without `MANDO_RADAR` does not enable lateral sanity.
- Default `match_vision_to_track(...)` behavior is unchanged.
- `check_lateral=True` rejects a wrong-lane track.
- `check_lateral=True` accepts a laterally consistent track.

Full pytest was blocked on this Windows/WSL machine because available built `.so` artifacts were from the comma/aarch64 environment and could not be imported on x86.
