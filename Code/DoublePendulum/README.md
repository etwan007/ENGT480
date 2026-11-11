# Double swing-up for the V2 arms

`PythonMaster2_swingup_V2.py` is a copy of the team's `PythonMaster2_swingup.py`, taken from the Pi's 2026-10-28 version at `etwan007/ENGT480` `experiment/legacy-pi-recovery` ba019f9. The original is not changed. The copy adds:
- `SYSID_RATIOS` set to the V2-arm fit from 2026-11-04 (`P1 5.2253, P2 2.0148, J2r 0.4506, H 0.4027, B1 0.0182, B2 0.0138`).
- Plan files `double_swingup_V2_<name>.json`. Choose one with `--path <name>` or with `SWING_PATH` at the top of the script. `--list` shows all of them.
- `ARM2_CHANNEL = "auto"`, which reads the joint encoder (TX2) from receiver slot 2 or slot 3. TX2 was on slot 3 on 2026-11-04.
- `PLAN_TRACK_M 0.30`. The swing still aborts at 0.36 m, and the soft limit stays at 0.40 m.

Everything else is unchanged, including tracking gains, balance, recovery, limits and CSV logging.

## V2b: radio-aware swing observer (use this one)

The first rig run of `same_4.0s` (2026-11-04, `swingup_double_20261104_153309_280984.csv`) failed all 5 attempts: each tracked the plan to within about 2 deg for 1.0-1.5 s, then drifted off. What the log shows:
- The V2 model is fine. A refit on the swing data moves the ratios by less than 4 %, and the fit error stays the same.
- The radio is the problem. About 11 % of TX1 frames and 5 % of TX2 frames are 10-70 ms old (same as in the excite run), and a reading's true age is off by about 2-3 ms. On a fast arm (5-15 rad/s) that is 1-3 deg of noise. The V2 observer assumed 0.07 deg, so it chased the noise. The cart then hit the 16 m/s² limit and the 0.83 m/s speed cap, and the arms drifted off the plan.
- On the twin with a radio fitted to that log, V2 also fails (7 % of attempts catch). With a clean radio it catches every time.

`PythonMaster2_swingup_V2b.py` is V2 with only these changes. V2 is unchanged.
- The swing observer trusts a reading less when the arm is fast (`SWING_TIMING_NOISE_MS = 3.0`) and skips readings it has already used.
- `SWING_DISTURBANCE_RAD_S2` goes from 3 to 10, and `SWING_ACCEL_M_S2` from 5 to 4.
- A radio gap of more than 5 frames during a swing now ends only that attempt (it recovers and retries). While settling, up to `MAX_SETTLE_MISSES = 12` missing frames are allowed. The V2 run stopped on a 100 ms gap while settling.

Twin check of `same_4.0s` [SIM] (`tools/twin_v2b_check.py`, results in `tools/twin_v2b_check.json`):

| Case | V2: attempts caught | V2b: attempts caught |
|---|---|---|
| Rig-like radio (18 runs) | 4/54 (7 %) | 14/40 (35 %) |
| Clean radio, nominal / arm-2 inertia +10 % / arm-2 mass -4 % | 12/12 | 12/12 |

Earlier seed sets gave V2b 21-75 % with the rig-like radio, against 2-6 % for V2. A better radio link would help more than any code change. Check that the TX antennas are attached and the LiPos are charged, and give the receiver a clear line of sight to the arms.

## Plans

The digital-twin column runs this exact script against the emulated Arduino, receiver, stepper and arms. Each plan has 11 cases × 3 seeds = 33 runs. The cases are nominal, arm-2 mass ±4 %, arm-2 inertia ±10 %, arm-1 inertia +10 %, arm-2 CoM +3 %, joint or pivot friction ×3, TX2 on slot 3, and 2 % radio loss with 3 ms latency.

| `--path` | Time | Cart | Speed | Accel | Twin: caught first try |
|---|---|---|---|---|---|
| `same_4.0s` (default) | 4.0 s | ±300 mm | 0.68 m/s | 4.7 m/s² | 33/33 |
| `same_3.5s` | 3.5 s | ±300 mm | 0.71 m/s | 5.1 m/s² | 33/33 |
| `same_3.0s` | 3.0 s | ±290 mm | 0.75 m/s | 6.4 m/s² | 30/33 (fails at arm-2 inertia +10 %) |
| `whip_4.5s` | 4.5 s | ±219 mm | 0.70 m/s | 4.9 m/s² | 28/33 (fails at arm-2 inertia +10 %, some at arm-2 mass -4 %) |
| `whip_3.5s` | 3.5 s | ±300 mm | 0.75 m/s | 6.6 m/s² | 27/33 (fails at arm-2 inertia ±10 %) |
| `whip_3.0s` | 3.0 s | ±263 mm | 0.75 m/s | 7.7 m/s² | 23/33 (fails at arm-2 inertia ±10 %, arm-2 mass -4 %) |
| `folded_4.5s` | 4.5 s | ±213 mm | 0.75 m/s | 6.8 m/s² | 24/33 (fails at arm-2 inertia ±10 %, arm-2 mass -4 %) |

In the whip plans arm 2 makes 1.5 turns before stopping upright. In the folded plan the arms turn opposite ways. All 7 plans catch every time in the nominal case, with friction ×3, with TX2 on slot 3, and with radio loss. The whip and folded plans are more sensitive to errors in arm 2's mass and inertia.

All results are simulation only [SIM]. Start with `same_4.0s`, then `same_3.5s`, before trying a whip plan.

## Before running
- **The cart travels up to ±300 mm**, and an attempt aborts at ±360 mm. The track was measured on 2026-11-04 at about 1080 mm switch to switch (±540 mm), so this fits when the cart starts within about 50 mm of the middle.
- Centre the cart before starting. Both arms must hang still.

## Install on the Pi (copy-paste)
```
mkdir -p ~/Desktop/DoubleSwingUpV2 && cd ~/Desktop/DoubleSwingUpV2
B=https://raw.githubusercontent.com/TheBarnyard/ChatGPT-Pendulum-Project/claude/project-thread-74rdr9/DoubleSwingUp_V2
for f in $(curl -fsSL $B/FILES); do curl -fsSLO $B/$f; done
sha256sum -c SHA256SUMS
```
## Run
```
cd ~/Desktop/DoubleSwingUpV2
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2b.py --list
$HOME/PythonLibraries/bin/python PythonMaster2_swingup_V2b.py --path same_4.0s
```
`PythonMaster2_swingup_V2.py` still works the same way, for comparison.
Other plans: `--path same_3.5s`, `same_3.0s`, `whip_4.5s`, `whip_3.5s`, `whip_3.0s`, `folded_4.5s`.
The prompts are the same as the original script's. CSV logs go to `~/Desktop/DoubleSwingUpV2/run_logs/`.

## Debug: angle sensors
`AngleSensorDiagnostic.py` is an unchanged copy of the Pi's `SinglePendulum/AngleSensorDiagnostic.py` (ba019f9). It uses `angle_protocol.py` from this folder. Motor power OFF. It reads only the receiver and prints all 3 slots five times a second: raw angle, change since start, age and dropouts. Move one arm at a time to see which slot it is on. Ctrl+C stops it.
```
cd ~/Desktop/DoubleSwingUpV2
$HOME/PythonLibraries/bin/python AngleSensorDiagnostic.py
```
