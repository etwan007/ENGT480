# ENGT480 Inverted Pendulum Project

[![Documentation](./assets/Documentation-Website-blue.svg)](https://etwan007.github.io/ENGT480/)

## Contact

For questions, collaboration, or inquiries regarding the current project, please contact:

Ethan Fuller: <etwan007@gmail.com>

## Contributors & Acknowledgments

This project builds upon the original two-stage inverted pendulum system developed by previous capstone contributors. Their work serves as the foundation for the current capstone project.

Original Repository: [cdlugos03/ENGT481](https://github.com/cdlugos03/ENGT481)

Previous Contributors:

- Samuel Whitesell: <whitesell.samuel@gmail.com>
- Ryan Hanshew: <hanshew.ryan@gmail.com>
- Patrick Clark: <Patrickgclark09@gmail.com>
- Edward Morris: <emmorris853@gmail.com>
- Cooper Dlugos: <Coopersd036@gmail.com>


---

## Team workflow and Testing

The Pi has two separate folders:

```text
Pi Desktop/
├── ENGT480/                    ← Main version
└── Experiments/
    ├── Ethan_EXP/              ← Ethan’s experiment
    └── Jaren_EXP/              ← Jaren’s experiment
```

- **ENGT480** follows the GitHub `main` branch. This is the approved version we tested and normally run.
- **Experiments** holds separate copies of the repo for work in progress.
- Each experiment follows its own GitHub branch and includes the whole repo, using the shared code from that branch's version.
- Changing an experiment does not change the main version or anyone else's experiment.

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

See the [installed Desktop setup](GeneralDocumentation/PI_AUTOMATION_SETUP.md) for shortcuts and verified backup coverage. For AI assistants and implementation details, see the [repository and Pi workflow reference](GeneralDocumentation/REPOSITORY_AND_PI_WORKFLOW.md) and [automation operating guide](Tools/PiAutomation/README.md).
