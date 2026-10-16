# Archived double-pendulum files

Moved on October 16, 2026. All four original files were moved without changing their contents.

The main controller is `../PythonMaster2_auto.py`. Automated tests are in `../Tests/`.

| Archived file | Why it was moved |
| --- | --- |
| [Test/PythonMasterControl2.py](Test/PythonMasterControl2.py) | Historical controller snapshot using six-byte packets; incompatible with receiver protocol v1 and not used by PythonMaster2_auto.py. |
| [Test/PythonMasterMath2.py](Test/PythonMasterMath2.py) | Historical model/tuning variant; kept with its matching matrices and model cache for reproducibility. |
| [Test/control_matrices2.txt](Test/control_matrices2.txt) | Historical matrices and gains, different from the active controller data; preserved with their generator. |
| [Test/symbolic_model2.pkl](Test/symbolic_model2.pkl) | Cached symbolic model from the historical Test folder; retained with that complete snapshot. |

## Original SHA-256 checksums

- `Test/PythonMasterControl2.py`: `70b4ef33008dced23a3cfb2b373593da97c6a6537ba2d91a5ea1062ccbb8973c`
- `Test/PythonMasterMath2.py`: `9a0fafdfe96c59f8a9cf6e8d08683b763f312e7c536660a5bf5bfb9fb0f68cb2`
- `Test/control_matrices2.txt`: `3b11d9ba0b09c095f4e65a44006a60cce8a6c7b08145b9b25a253e47ac47e116`
- `Test/symbolic_model2.pkl`: `64c3bbe5e3aaf5486fa2460fb3cbb2063e3e67cf392e6e5ded0b3ba4b86b9fb3`
