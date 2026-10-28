# Recovered Raspberry Pi experiments

This branch preserves the old Pi project from `/home/engt481/Desktop/GitHub Repository/ENGT481` as it existed on October 28, 2026, including unpublished edits, experimental variants, model files, firmware, and dated backups.

All 84 recovered files are byte-for-byte copies. Their original relative paths are retained below this folder; the old root README is saved as [README.original.md](README.original.md). The current repository code and automation remain at the repository root.

## Start here

| Work | Recovered files |
|---|---|
| Double swing-up | [PythonMaster2_swingup.py](Code/DoublePendulum/PythonMaster2_swingup.py), [trajectory](Code/DoublePendulum/double_swingup_trajectory.json) |
| Double balancing | [PythonMaster2_auto.py](Code/DoublePendulum/PythonMaster2_auto.py), [model generator](Code/DoublePendulum/PythonMasterMath2.py), [matrices](Code/DoublePendulum/control_matrices2.txt) |
| System identification | [PendulumSysID.py](Code/DoublePendulum/PendulumSysID.py), [analyze_sysid.py](Code/DoublePendulum/analyze_sysid.py), [apply_sysid.py](Code/DoublePendulum/apply_sysid.py) |
| Double controller variants | [DoublePendulum folder](Code/DoublePendulum/) contains Claude, damping, latency, and no-trim versions |
| Single balancing and swing-up | [SinglePendulum folder](Code/SinglePendulum/) contains auto, swing-up, KCP, Claude, and Logan variants |
| Shared helpers | [Global folder](Code/Global/), plus the original local helper copies beside the controllers |
| Receiver and transmitters | [Wireless folder](Code/Wireless/) |
| Motor slave and checks | [ArduinoSlave folder](Code/ArduinoSlave/) |
| Historical models and wiring | [MATLAB Simulation](MATLAB%20Simulation/), [Wiring](Wiring/) |

## Using the recovered project

Treat `LegacyPi/` as the recovered project root, and use its controller folders when working with these versions. Keep the recovered helper modules, matrices, trajectory, and firmware together. The root-level `Code/` folder contains the newer main-branch code.

These scripts retain their original behavior, serial settings, tuning values, and any machine-specific paths. Several scripts operate the motor or modify model/configuration files. Inspect the chosen entry point and hardware setup before running it. The recovery did not launch a controller, flash firmware, regenerate models, or install the older uploader.

CSV run logs and Python caches were excluded from this source branch; 379 generated files were left in the original checkout. The separate `logs` branch and existing uploader remain the log-backup workflow. This recovery does not claim that every historical run log has been uploaded.

## Verification

- All 84 file copies were compared with their originals and verified by SHA-256.
- All 53 recovered Python files passed syntax parsing without importing or executing controllers.
- The source checkout and its local changes were preserved.
- Hardware operation and firmware builds were not tested.
- [MANIFEST.json](MANIFEST.json) records the source commit, main-branch base, every source/destination path, byte size, and SHA-256.

## Complete file index

- [Code/ArduinoSlave/ArduinoSlave.ino](Code/ArduinoSlave/ArduinoSlave.ino)
- [Code/ArduinoSlave/StepCountCheck.py](Code/ArduinoSlave/StepCountCheck.py)
- [Code/CommunicationTest.py](Code/CommunicationTest.py)
- [Code/DoublePendulum/Archive/PythonMaster2_autotest.py](Code/DoublePendulum/Archive/PythonMaster2_autotest.py)
- [Code/DoublePendulum/Archive/PythonMasterControl2.py](Code/DoublePendulum/Archive/PythonMasterControl2.py)
- [Code/DoublePendulum/Archive/README.md](Code/DoublePendulum/Archive/README.md)
- [Code/DoublePendulum/Archive/Test/PythonMasterControl2.py](Code/DoublePendulum/Archive/Test/PythonMasterControl2.py)
- [Code/DoublePendulum/Archive/Test/PythonMasterMath2.py](Code/DoublePendulum/Archive/Test/PythonMasterMath2.py)
- [Code/DoublePendulum/Archive/Test/control_matrices2.txt](Code/DoublePendulum/Archive/Test/control_matrices2.txt)
- [Code/DoublePendulum/Archive/Test/symbolic_model2.pkl](Code/DoublePendulum/Archive/Test/symbolic_model2.pkl)
- [Code/DoublePendulum/PendulumSysID.py](Code/DoublePendulum/PendulumSysID.py)
- [Code/DoublePendulum/PythonMaster2_auto.py](Code/DoublePendulum/PythonMaster2_auto.py)
- [Code/DoublePendulum/PythonMaster2_swingup.py](Code/DoublePendulum/PythonMaster2_swingup.py)
- [Code/DoublePendulum/PythonMaster2_swingup.py.bak](Code/DoublePendulum/PythonMaster2_swingup.py.bak)
- [Code/DoublePendulum/PythonMaster2_swingup.py.bak_20261027_182719](Code/DoublePendulum/PythonMaster2_swingup.py.bak_20261027_182719)
- [Code/DoublePendulum/PythonMaster2_swingup.py.bak_20261027_184231](Code/DoublePendulum/PythonMaster2_swingup.py.bak_20261027_184231)
- [Code/DoublePendulum/PythonMaster2_swingup_old_20261021_165257.py](Code/DoublePendulum/PythonMaster2_swingup_old_20261021_165257.py)
- [Code/DoublePendulum/PythonMasterDouble-Claude.py](Code/DoublePendulum/PythonMasterDouble-Claude.py)
- [Code/DoublePendulum/PythonMasterDouble-Damping30.py](Code/DoublePendulum/PythonMasterDouble-Damping30.py)
- [Code/DoublePendulum/PythonMasterDouble-Damping45-Latency8.py](Code/DoublePendulum/PythonMasterDouble-Damping45-Latency8.py)
- [Code/DoublePendulum/PythonMasterDouble-Damping45-NoTrim.py](Code/DoublePendulum/PythonMasterDouble-Damping45-NoTrim.py)
- [Code/DoublePendulum/PythonMasterDouble-Damping45.py](Code/DoublePendulum/PythonMasterDouble-Damping45.py)
- [Code/DoublePendulum/PythonMasterMath2.py](Code/DoublePendulum/PythonMasterMath2.py)
- [Code/DoublePendulum/analyze_sysid.py](Code/DoublePendulum/analyze_sysid.py)
- [Code/DoublePendulum/angle_packet_fallback.py](Code/DoublePendulum/angle_packet_fallback.py)
- [Code/DoublePendulum/angle_protocol.py](Code/DoublePendulum/angle_protocol.py)
- [Code/DoublePendulum/apply_sysid.py](Code/DoublePendulum/apply_sysid.py)
- [Code/DoublePendulum/control_matrices2.txt](Code/DoublePendulum/control_matrices2.txt)
- [Code/DoublePendulum/double_swingup_trajectory.json](Code/DoublePendulum/double_swingup_trajectory.json)
- [Code/DoublePendulum/double_swingup_trajectory.json.bak_20261027_182719](Code/DoublePendulum/double_swingup_trajectory.json.bak_20261027_182719)
- [Code/DoublePendulum/double_swingup_trajectory.json.bak_20261027_184231](Code/DoublePendulum/double_swingup_trajectory.json.bak_20261027_184231)
- [Code/DoublePendulum/run_csv_logger.py](Code/DoublePendulum/run_csv_logger.py)
- [Code/DoublePendulum/symbolic_model2.pkl](Code/DoublePendulum/symbolic_model2.pkl)
- [Code/Global/AngleSensorDiagnostic.py](Code/Global/AngleSensorDiagnostic.py)
- [Code/Global/LOG_UPLOAD.md](Code/Global/LOG_UPLOAD.md)
- [Code/Global/README.md](Code/Global/README.md)
- [Code/Global/Tests/test_log_upload.py](Code/Global/Tests/test_log_upload.py)
- [Code/Global/__init__.py](Code/Global/__init__.py)
- [Code/Global/angle_packet_fallback.py](Code/Global/angle_packet_fallback.py)
- [Code/Global/angle_protocol.py](Code/Global/angle_protocol.py)
- [Code/Global/install_log_upload.py](Code/Global/install_log_upload.py)
- [Code/Global/live_offset_keys.py](Code/Global/live_offset_keys.py)
- [Code/Global/physical_parameters.py](Code/Global/physical_parameters.py)
- [Code/Global/restore_csv_log.py](Code/Global/restore_csv_log.py)
- [Code/Global/run_csv_logger.py](Code/Global/run_csv_logger.py)
- [Code/Global/upload_csv_logs.py](Code/Global/upload_csv_logs.py)
- [Code/Logan_test.py](Code/Logan_test.py)
- [Code/Old_Code/Arduino_PID_Loop/PIDtest/PIDtest.ino](Code/Old_Code/Arduino_PID_Loop/PIDtest/PIDtest.ino)
- [Code/SignalTest.py](Code/SignalTest.py)
- [Code/Single/PendulumExcite/PendulumExcite.py](Code/Single/PendulumExcite/PendulumExcite.py)
- [Code/SinglePendulum/AngleSensor.py](Code/SinglePendulum/AngleSensor.py)
- [Code/SinglePendulum/AngleSensorDiagnostic.py](Code/SinglePendulum/AngleSensorDiagnostic.py)
- [Code/SinglePendulum/Logans_PythonMaster.py](Code/SinglePendulum/Logans_PythonMaster.py)
- [Code/SinglePendulum/PythonMaster.py](Code/SinglePendulum/PythonMaster.py)
- [Code/SinglePendulum/PythonMasterSwingUpExp.py](Code/SinglePendulum/PythonMasterSwingUpExp.py)
- [Code/SinglePendulum/PythonMasterSwingUpExp2.py](Code/SinglePendulum/PythonMasterSwingUpExp2.py)
- [Code/SinglePendulum/PythonMasterSwingUpExpKCP.py](Code/SinglePendulum/PythonMasterSwingUpExpKCP.py)
- [Code/SinglePendulum/PythonMasterUpExpClaude.py](Code/SinglePendulum/PythonMasterUpExpClaude.py)
- [Code/SinglePendulum/PythonMasterUpExpClaudeImproved.py](Code/SinglePendulum/PythonMasterUpExpClaudeImproved.py)
- [Code/SinglePendulum/PythonMaster_auto.py](Code/SinglePendulum/PythonMaster_auto.py)
- [Code/SinglePendulum/PythonMaster_autotest.py](Code/SinglePendulum/PythonMaster_autotest.py)
- [Code/SinglePendulum/TestingDataForDigitalTwin-Claude.py](Code/SinglePendulum/TestingDataForDigitalTwin-Claude.py)
- [Code/SinglePendulum/angle_packet_fallback.py](Code/SinglePendulum/angle_packet_fallback.py)
- [Code/SinglePendulum/angle_protocol.py](Code/SinglePendulum/angle_protocol.py)
- [Code/SinglePendulum/encoder_diagnostic.py](Code/SinglePendulum/encoder_diagnostic.py)
- [Code/SinglePendulum/live_offset_keys.py](Code/SinglePendulum/live_offset_keys.py)
- [Code/SinglePendulum/run_csv_logger.py](Code/SinglePendulum/run_csv_logger.py)
- [Code/SinglePendulum/swingup_control.py](Code/SinglePendulum/swingup_control.py)
- [Code/Wireless/ESP32_Code/DebugReceiverMaster/DebugReceiverMaster.ino](Code/Wireless/ESP32_Code/DebugReceiverMaster/DebugReceiverMaster.ino)
- [Code/Wireless/ESP32_Code/Receivermaster/Receivermaster.ino](Code/Wireless/ESP32_Code/Receivermaster/Receivermaster.ino)
- [Code/Wireless/ESP32_Code/Receivermaster/Receivermaster.ino.backup](Code/Wireless/ESP32_Code/Receivermaster/Receivermaster.ino.backup)
- [Code/Wireless/ESP32_Code/Receivermaster/angle_protocol.h](Code/Wireless/ESP32_Code/Receivermaster/angle_protocol.h)
- [Code/Wireless/Transmitters/TX1/TX1.ino](Code/Wireless/Transmitters/TX1/TX1.ino)
- [Code/Wireless/Transmitters/TX2/TX2.ino](Code/Wireless/Transmitters/TX2/TX2.ino)
- [MATLAB Simulation/DiscreteSinglePendulum.m](MATLAB%20Simulation/DiscreteSinglePendulum.m)
- [MATLAB Simulation/PIDsim.m](MATLAB%20Simulation/PIDsim.m)
- [MATLAB Simulation/PolePlacementTest.m](MATLAB%20Simulation/PolePlacementTest.m)
- [MATLAB Simulation/RK4testing.m](MATLAB%20Simulation/RK4testing.m)
- [MATLAB Simulation/SinglePendulumMath.m](MATLAB%20Simulation/SinglePendulumMath.m)
- [MATLAB Simulation/SinglePendulumSim.m](MATLAB%20Simulation/SinglePendulumSim.m)
- [MATLAB Simulation/StateEstimation.m](MATLAB%20Simulation/StateEstimation.m)
- [MATLAB Simulation/cp_sim_2.slx](MATLAB%20Simulation/cp_sim_2.slx)
- [README.original.md](README.original.md)
- [Wiring/Wire_Diagram.png](Wiring/Wire_Diagram.png)
