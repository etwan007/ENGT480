# Improved double-pendulum controller (experiment)

Swing-up from hanging → LQR catch → balance, with recovery/retry and e-stops.
The full write-up is in [Code/DigitalTwin/README.md](../../DigitalTwin/README.md):
video and log findings, twin parameters, Monte Carlo results, and the
**first hardware test checklist**. Read the checklist before running this on the rig.

```sh
# digital twin (laptop or Pi; numpy + matplotlib)
python3 PythonMaster2_improved.py --sim --duration 20
python3 PythonMaster2_improved.py --sim --balance-only
# rig (find the checkout ID with `engt status`)
engt run experiment/claude-controller-twin Code/DoublePendulum/Improved/PythonMaster2_improved.py
engt run experiment/claude-controller-twin Code/DoublePendulum/Improved/PythonMaster2_improved.py --balance-only
```

Files: `improved_controller.py` (controller, NumPy only), `improved_gains.json`
(`design_lqr.py`), `swingup_plan.json` (`swingup_trajectory.py`),
`plan_model.py`, `stepper_mapping.py`, `PythonMaster2_improved.py` (entry
script; reuses `../PythonMaster2_auto.py` and `Code/Global/`). Run CSVs are
written to `Code/DoublePendulum/run_logs/improved_double_*.csv`.

Tests: `python3 -m unittest discover -s Tests -t . -p 'test_*.py'` (from this folder).
