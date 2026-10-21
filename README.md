# ENGT480

Repository for ENGT 480/481 Senior Design Capstone (Triple Inverted Pendulum project) Continued from    Capstone group 2025-2026. Credit for first round of commit files go to:
    - Samuel Whitesell: whitesell.samuel@gmail.com
    - Ryan Hanshew: hanshew.ryan@gmail.com
    - Patrick Clark: Patrickgclark09@gmail.com
    - Edward Morris: emmorris853@gmail.com
    - Cooper Dlugos: Coopersd036@gmail.com

## ENGT480 workflow — simple team guide

The Pi has two separate folders:

```text
Pi Desktop/
├── ENGT480/                    ← Main version
└── Experiments/
    ├── Ethan_EXP/              ← Ethan’s experiment
    └── Jaren_EXP/              ← Jaren’s experiment
```

- **ENGT480** follows the GitHub `main` branch. This is the approved version we normally run.
- **Experiments** holds separate copies of the repo for work in progress.
- Each experiment follows its own GitHub branch and includes the whole repo, using the shared code from that branch’s version.
- Changing an experiment does not change the main version or anyone else’s experiment.

### How to work on something

1. **Create a branch** with a descriptive name ending in `_EXP` or starting with `experiment/`, such as `Ethan_wirelessTX_EXP`. The example name `Ethan-wirelessTX` is also allowlisted.
2. **Edit the normal files on that branch.** You do not need to create an `Experiments` folder inside the repo.
3. **Commit and push your branch to GitHub.** The Pi creates the corresponding experiment folder during the next eligible update.
4. **Test from your experiment folder on the Pi.** Later pushes will update that folder between runs.
5. **Commit and push any changes made directly on the Pi**, so they are available on your computer too.
6. **When ready, open a pull request into `main`.** Review and merge the changes.
7. **The Pi’s ENGT480 folder updates between runs**, bringing in the merged changes.

### Short terminal commands

The terminal commands work from any directory on the Pi and do the same jobs as the Desktop shortcuts. To use the short name in your current terminal, run:

```sh
alias engt='python3 ~/.local/lib/engt480-code-sync/pi_sync.py'
```

To keep this shortcut in future terminal sessions, open `nano ~/.bashrc`, add that same alias line, save, and exit. Then reload the settings:

```sh
source ~/.bashrc
```

This alias is a terminal convenience. It is installed for the shared Pi account; adding these instructions to the repository does not install it automatically on other accounts.

| Command | What it does |
| --- | --- |
| `engt run` | Asks which checkout and script to run. Uses the configured PythonLibraries environment and holds the rig/update locks until the script stops. No separate environment activation is needed. |
| `engt run Ethan_EXP Code/SinglePendulum/PythonMaster_auto.py` | Starts that specific script in Ethan_EXP with the same environment and run protection. |
| `engt publish "Describe your changes"` | From inside an experiment checkout, previews saved source changes and unpublished commits, asks for confirmation, commits, and pushes to that branch. Logs/caches stay local; pause settings are preserved. Main/logs publishing is refused. |
| `engt status` | Shows registered folders, their branches, and the latest sync results or reported problems. |
| `engt pause` | Pauses code updates for all checkouts. It waits for any current sync to finish; it does not stop controllers or CSV backups. |
| `engt resume` | Clears the global pause so the timer can check again. Local edits, active runs, open Thonny sessions, and checkout-specific pauses can still defer updates. |
| `engt pause Ethan_EXP` | Pauses updates only for the Ethan_EXP checkout. |
| `engt resume Ethan_EXP` | Clears that checkout's pause. A global pause still takes precedence. |
| `engt sync` | Requests a sync pass now, using the same locks and local-edit protections as the timer. It does not force updates or resolve conflicts. |
| `engt sync --dry-run` | Previews branch discovery and intended checkout creation/checks without fetching, creating checkouts, or changing their contents. Operational lock/state directories may be created. |

Replace `Ethan_EXP` with the checkout ID shown by `engt status` when working in another experiment. Close Thonny before using `engt run`; the launcher refuses to start while Thonny is open or a recognized hardware serial port is already in use.

For direct nano/Thonny editing or direct Python launches, pause updates first. Saved local edits make the updater leave the checkout alone, but unsaved editor buffers are invisible to Git, and a direct Python launch does not acquire the shared run lock. The protected `engt run` command handles run locking automatically. Commit and push Pi edits using your contributor Git identity/authentication before resuming updates, or leave unfinished work saved and the checkout pending. CSV backups continue while code updates are paused.

To publish from another directory, specify the checkout:

```sh
engt publish "Tune transmitter timing" --checkout Ethan_EXP
```

Save your files before publishing. The command uses the checkout's effective Git author and the Pi's configured repository SSH key. It will stop if GitHub has newer/diverged history; it never automatically pulls, rebases, or force-pushes. A failed push leaves the commit saved locally, and rerunning the same command retries it without creating an empty duplicate commit. The background updater still only downloads code; source publishing happens only when you explicitly run this command.

The `engt` alias is now saved in this Pi account's `~/.bashrc`. Open a new terminal or run `source ~/.bashrc` to load it in an existing session. Other accounts still need to add the alias themselves.

### Branch naming rules

Names are case-sensitive. Use a descriptive person/task name with one of these automatic enrollment patterns:

| Pattern | Examples | Pi behavior |
| --- | --- | --- |
| Ends with `_EXP` | `Ethan_EXP`, `Jaren_EXP`, `Ethan_wirelessTX_EXP` | Creates or updates an experiment checkout automatically after the branch is pushed. `_EXP` must be uppercase. |
| Starts with `experiment/` | `experiment/ethan-wirelessTX`, `experiment/jaren-swingup` | Creates or updates an experiment checkout automatically after the branch is pushed. |
| Explicitly allowlisted name | `Ethan-wirelessTX` | This exact example name is already supported. Other names require an entry in the Pi configuration's `extra_branches` list. |

- A branch created only on a laptop will not appear on the Pi until it is pushed to `etwan007/ENGT480` on GitHub.
- `main` is reserved for `~/Desktop/ENGT480`; `logs` is reserved for CSV backups. Neither becomes an experiment checkout.
- Other branch names are ignored unless allowlisted. For example, `Ethan-wirelessTX` is a specific exception; arbitrary names ending in `-wirelessTX` are not a naming pattern.
- Simple safe names become the same folder name under `~/Desktop/Experiments`. Names with slashes or unsupported characters get a readable folder name plus a stable digest. Use `engt status` to find the actual folder and checkout ID.
- Each branch gets one complete repo checkout. Later pushes update that same folder; they do not create a new folder for every commit. You do not need to commit an `Experiments` directory inside the repo.
- Removing a remote branch does not delete its Pi folder or logs. Retained registered folders continue to receive CSV backups.

### Team rules

- One person edits a given experiment checkout at a time.
- Only one person operates the physical rig at a time.
- Use the **ENGT480 Run** Desktop shortcut for hardware runs. It holds the shared rig and update locks.
- Use **ENGT480 Pause Updates** before editing on the Pi or using Thonny directly; save your work and close stale editor buffers before resuming. Updates also defer while Thonny is open.
- Automatic updates wait if that checkout has local edits or a managed controller is active.
- Downloading an experiment never starts it automatically.
- Before switching computers, commit and push your work. On the other computer, pull the latest changes before editing.

See the [installed Desktop setup](https://github.com/etwan007/ENGT480/blob/Ethan_EXP/GeneralDocumentation/PI_AUTOMATION_SETUP.md) for shortcuts and verified backup coverage. For AI assistants and implementation details, see the [repository and Pi workflow reference](https://github.com/etwan007/ENGT480/blob/Ethan_EXP/GeneralDocumentation/REPOSITORY_AND_PI_WORKFLOW.md) and [automation operating guide](https://github.com/etwan007/ENGT480/blob/Ethan_EXP/Tools/PiAutomation/README.md).

FROM LAST YEAR <

        CAD FILES ARE ON THE DESKTOP OF THE RASPBERRY PI

        **Raspberry Pi setup**
        - The python libraries are installed in a venv virtual environment called 'PythonLibraries'.
        - path: /home/engt481/PythonLibraries
        - Thonny is set up to use the python libraries from this environment, if you want to use a different editor you'll need to set this up.
        - Activation command for running scripts in terminal (from engt481 directory): source PythonLibraries/bin/activate
        - Deactivation command: deactivate

        **Single pendulum LQG (Linear Quadratic Gaussian) setup:**
        - Run communication test program and hold up pendulum to get a rough estimate of the value when straight up.
        - If the angle is close to the rollover point (near 0 or 16383), rotate the pendulum about the shaft so the rollover point will be near pointing down.
        - replace the value for the 'correction' variable with the measured value for straight up.
        - start with the 'ANGLE CORRECTION', and 'ANGLE TUNING' lines enabled, and run the program until the pendulum centers near the middle (it may oscillate a bit).
        - Comment out the 'ANGLE CORRECTION' line, and replace the 'correction' variable with the final printed correction value. Run again until the pendulum is centered.
        - Comment out the 'ANGLE TUNING' lines, and run again with the new correction value. Make sure that it centers itself very close to the starting point.
        - The first pendulum angle should be tuned at this point, and be usable for the any of the control scripts.

        **Double pendulum LQG setup:**
        - Use the single pendulum script first to tune the first pendulum angle
        - Run the communication test program, and let the pendulums hang. Record the second pendulum angle to use for the 'correction2' variable value.
        - The control script should be ready to run at this point.
        - To adjust tuning, or physical measurements of the pendulum parts, update the physical measurements in `Code/Global/physical_parameters.py` (or tuning in the math script), and run `PythonMasterMath2.py`. this will update the control_matrices2.txt file.
        - To adjust the abstract dynamics, make sure to delete the .pkl file, and then run the math script to generate a file for the new dynamics. This will take a long time to run (only when re-generating the .pkl file), so run this on a personal computer. it will take from 15-30 minutes to calculate.
        - The pendulum will likely not stabilize on the first test, so methodically mess with the 'correction2' value until the pendulum is able to stay upright (Increasing the value brings it left, decreasing brings it right)
        - If the pendulum is slowly drifting off, use a finger to lightly touch the 2nd pendulum prevent it from going off the rail while it tunes. It should slowly correct 'correction' until the pendulum begins to center itself. It will oscillate at first, but should stabilize eventually in the center.

        **Triple Pendulum LQG:**
        We had not attempted this as of writing this. Good luck! This will require a more sophisticated control system than linear LQG. Here are some ideas we had:
        - Could try re-linearizing the system every time step to approximate a non-linear system with LQG. (Raspberry pi may not be powerful enough if not well optimized)
        - Could try model predictive control (MPC). (Raspberry pi may not be powerful enough if not well optimized)
        - Could try a machine learning approach. 


        **Things to be careful of:**
        - Printing to terminal takes a surprising amount of time. Minimize print statements in the control loop, and round printed float values. Sometimes angles being dropped and triggering the filter messages can be because the control loop took too long (because of printing), not actually because a bad angle was received from the encoder.
        - Make sure to power down the raspberry pi properly. If you just cut the power, it could cause it to corrupt the github. (This is fixable, just annoying to deal with)

        **Description of the Control System**

        The current control system uses LQG (Linear Quadratic Gaussian). This is just LQR (Linear Quadratic Regulator) with a Kalman Filter. The control loop starts by taking the previous measurements of the system, and estimating the current states. the states are a matrix that contain all the necessary information to describe the pendulum's motion in a given instant. For the single pendulum this contains the angle, angular velocity, cart position, and cart velocity. After this step, the raspberry pi waits for the angles to be sent over serial by the esp32. These actual measurements are used to update the kalman filter estimations and make them more accurate. Then the control loop calculates the control force to apply, and pulls the control velocity from the kalman filter estimation of the velocity. The kalman filter is useful because it not only filters out noise in the measurements, but it also estimates the states that aren't measured based on the dynamics of the system. The control loop uses the control velocity 'xDiv' to calculate the frequency to pulse the stepper motor, then converts that to control the 2560 hardware timer. It sends this to the arduino, which then outputs the control frequency. After sending this, the arduino returns the current position of the cart (It tracks this by counting steps sent to the motor with an interrupt), and the loop restarts. The order of the loop means that the position value will always be a loop old, but this doesn't really matter since the position is far less critical than the angles to proper balancing.
            
        The first part of the single pendulum code (or the math script for the double) start with the differential equations that describe the system. We are using lagrange derivations since they scale much more easily than dealing with forces and torques directly, and produce the exact same end result. T represents the total kinetic energy of the system, and U represents the potential energy. We found the double pendulum equations in a paper (linked under helpful resources), and triple pendulum equations are available in papers also. The script converts these equations into a state space model that is set up like this: Y_derivative(matrix of state derivatives) = a matrix of equations with variables for the states. The state space model is then linearized around the stabilization point (all angles, positions and velocities = 0), and the A and B matrix are created. The C and D matrices are input manually, and represent which states are estimated, and which are measured. The C matrix is and identity matrix with each row that represents an estimated state removed. The D matrix has an element for each measured state. 
            
        These linearized matrices are discretized. This converts them from 'Y_dot = A x Y + B x u' to 'Y_new = A x Ylast + B x u'. This slightly simplifies control loop calculations to keep from having to integrate Y_dot. These matrices are also used to calculate the kalman filter matrices and the LQR gain matrix.
            
        Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(100,1,100,1,20,10), 0.1)
        Kd is the LQR gain matrix, this is multiplied by the current state matrix to get the control force. the sp.diag(theta1_weight, w1_weight, theta2_weight, w2_weight, x_weight, xDiv_weight) contains values that represent how much the controller cares about keeping each state close to zero. The last value (0.1 in the example line) represents how much the controller tries to limit the control force (bigger values try to limit more). Smaller values mean the controller doesn't care much about managing that state.
        
        Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), 0.01*sp.diag(1,1,1))
        This line calculates the kalman filter matrices. sp.diag(theta1_weight, w1_weight, theta2_weight, w2_weight, x_weight, xDiv_weight) determines how much the controller trusts the measured values for each. The values for estimated states don't matter (these are set to 0.001 in the example line). A value of 1 fully trusts the estimated state, and a value of 0 fully trusts the measurement. You likely won't have a reason to mess with the tuning of this.
            
        Because this control system is linearized, the current system cannot just be scaled up for the triple pendulum, it won't be accurate enough.

        **Helpful resources:**
        - Matlab pendulum simulation tutorial:
        https://www.mathworks.com/help/symbolic/derive-and-simulate-cart-pole-system.html
        - Link to double pendulum paper where we found dynamic equations:
        https://www.researchgate.net/publication/348468205_Modeling_and_Control_of_a_Double_Inverted_Pendulum_using_LQR_with_Parameter_Optimization_through_GA_and_PSO

        **Suggestions:**
        - A way to tune the angles more precisely could be helpful. Maybe a program could be designed to help find the second pendulum angle more easily and precisely?
        - Bearing friction and air resistance are not included in the model. These may need to be modeled for the triple pendulum.
        - This is probably uneccesary, but if a more powerful control computer is needed (replacing the raspberry pi 4) it would likely be easy to switch it out. The python code is not raspberry pi specific (at least not that I am aware), and should be easy to adapt.

    >

**Contact**

If you have a question about the system that we forgot to address:
 - Ethan Fuller: etwan007@gmail.com


## Current calibrated controllers and run review

- Single pendulum: `Code/SinglePendulum/PythonMaster_auto.py`.
- Double pendulum: run `Code/DoublePendulum/PythonMaster2_auto.py` (the main controller) or `PythonMasterControl2.py` (a compatibility launcher); both start the same calibrated controller with receiver protocol v1. Its existing `control_matrices2.txt` supplies the model and gains. See [double-pendulum setup](Code/DoublePendulum/README.md).
- Both controllers have `SHOW_BALANCING_LOGS` and `SAVE_RUN_CSV` switches near the top. Set either to `False` to disable routine balancing output or CSV recording. Sensor errors and faults remain visible.
- The single controller supports live arrow-key offset adjustments; both controllers handle one unusable angle cycle with an estimator prediction and stop on repeated misses by default.
- Copy `Code/Global/` alongside the controller folders when deploying to the Pi; shared helpers are imported from that package.
- Open `Code/RunReview/balancing_visualizer.html` locally to replay a single- or double-pendulum CSV. Its example CSVs contain synthetic data.
- See [logging, offsets, and replay instructions](Code/RunReview/RUN_LOGGING_AND_VISUALIZER.md).

Hardware-free checks (Python with NumPy installed; Node.js for the visualizer):

```sh
python3 -m unittest discover -s Code/Global/Tests -p 'test_*.py'
(cd Code/SinglePendulum && python3 -m unittest discover -p 'test_*.py')
(cd Code/DoublePendulum && python3 -m unittest discover -s Tests -t . -p 'test_*.py')
(cd Code/RunReview && node test_balancing_visualizer.js)
```

The current calibrated controllers require **receiver serial protocol v1**.
Update the ESP32 receiver and Python readers together; TX1/TX2 keep their
existing radio format. See [receiver deployment and diagnostics](Code/Wireless/ANGLE_PROTOCOL.md).
Older controllers and `CommunicationTest.py` still use the legacy serial format
and must not be run with the updated receiver. The Arduino motor protocol is unchanged.


Shared Python configuration and utilities now live in `Code/Global/`.
Deploy the entire `Global` folder as a sibling of `SinglePendulum` and
`DoublePendulum`; see [shared directory guide](https://github.com/etwan007/ENGT480/blob/Ethan_EXP/Code/Global/README.md).
