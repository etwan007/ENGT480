# Archived double-pendulum files

Moved on October 16, 2026. Every original file was preserved without changing its contents.

The active controller is ../PythonMaster2_auto.py. Its helpers, active model tools, control_matrices2.txt, and run_logs remain in the parent folder. Archived controllers use the old six-byte protocol and must not run with the v1 receiver.

| Archived file | Why it was moved |
| --- | --- |
| [PythonMasterControl2.py](PythonMasterControl2.py) | Legacy six-byte controller; superseded by PythonMaster2_auto.py using receiver protocol v1. |
| [PythonMaster2_autotest.py](PythonMaster2_autotest.py) | Older alternate six-byte controller, not an automated test; not used by the current main script. |
| [Test/PythonMasterControl2.py](Test/PythonMasterControl2.py) | Historical six-byte controller snapshot; incompatible with the v1 receiver. |
| [Test/PythonMasterMath2.py](Test/PythonMasterMath2.py) | Historical model/tuning variant; retained with its own matrices and model cache. |
| [Test/control_matrices2.txt](Test/control_matrices2.txt) | Historical gains and matrices, different from active data; retained with their generator. |
| [Test/symbolic_model2.pkl](Test/symbolic_model2.pkl) | Historical model cache; retained with the complete Test snapshot. |

## Original SHA-256 checksums

- PythonMasterControl2.py: 2c1f517964d8070c1a4079c59ce9654018ec45825d186eec9dfdfea451050c39
- PythonMaster2_autotest.py: d7d41e2bae436dcd467083d36b872098a7cdb9e47d1a371a75544ce611f9211a
- Test/PythonMasterControl2.py: 70b4ef33008dced23a3cfb2b373593da97c6a6537ba2d91a5ea1062ccbb8973c
- Test/PythonMasterMath2.py: 9a0fafdfe96c59f8a9cf6e8d08683b763f312e7c536660a5bf5bfb9fb0f68cb2
- Test/control_matrices2.txt: 3b11d9ba0b09c095f4e65a44006a60cce8a6c7b08145b9b25a253e47ac47e116
- Test/symbolic_model2.pkl: 64c3bbe5e3aaf5486fa2460fb3cbb2063e3e67cf392e6e5ded0b3ba4b86b9fb3
