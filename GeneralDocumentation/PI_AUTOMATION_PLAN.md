# ENGT480 Pi automation implementation plan

Status: implementation is now available in [Tools/PiAutomation](../Tools/PiAutomation/README.md). See the [installed Desktop setup](PI_AUTOMATION_SETUP.md) for actual deployment and verification. The sections below preserve the original design checklist; finalized filenames, operating commands, and behavior are documented in those implementation references.

The implementation consolidates synchronization, status, pause/resume, and controller launching in `pi_sync.py`, uses the Pi Desktop as its workspace, and preserves the legacy `uploaded.json` confirmation schema by prefixing new source keys with stable backup namespaces. It does not require the proposed per-source state migration.

This extends the [repository and Pi workflow reference](REPOSITORY_AND_PI_WORKFLOW.md).

## 1. What the finished system should do

Once installed and enabled, the Pi should discover opted-in experiment branches, create one complete checkout for each branch, and update those checkouts after later pushes. A separate ENGT480 checkout should follow main. It should update after changes reach main, between runs.

Downloading code must never start a controller, install dependencies, flash firmware, regenerate control matrices, or run scripts supplied by an experiment branch. The updater must not commit, push, stash, reset, clean, or resolve conflicts automatically.

The intended layout is:

```text
<PI_WORKSPACE>/
├── ENGT480/                         # main
└── Experiments/
    ├── Ethan_EXP/                   # Ethan_EXP
    └── Ethan-wirelessTX/            # Ethan-wirelessTX, if enrolled

~/.local/lib/engt480-code-sync/       # Reviewed, installed automation
~/.config/engt480-code-sync/          # Machine-specific configuration
~/.local/state/engt480-code-sync/     # Registry, locks, status, history
```

`<PI_WORKSPACE>` is a placeholder. Do not assume the existing Pi checkout has already been renamed or moved.

## 2. Recommended choices

These are proposed defaults, not configuration already installed on the Pi.

| Item | Proposed choice | Reason |
| --- | --- | --- |
| GitHub repository | `etwan007/ENGT480` | Matches the repository used for the team's current work. Verify before setup. |
| Checkout method | Independent, single-branch clones | Straightforward for teammates to inspect, edit, commit, and push independently. |
| Branch enrollment | Names ending in `_EXP` or starting with `experiment/`, plus an explicit allowlist | Enrolls `Ethan_EXP` and supports future task branches without cloning every unrelated branch. |
| Existing example name | Allowlist `Ethan-wirelessTX` if that branch is used | Preserves the example name without requiring a rename. |
| Polling | About once per minute while eligible to sync | Works behind the Pi's existing network connection without an inbound webhook. |
| Main updates | Automatic fast-forward updates when clean, unpaused, and idle | Matches the agreed merge-to-main workflow. |
| Removed branches | Retain their local folders and mark them inactive | Preserves local edits and run data. |
| First installation | Services installed disabled; dry-run first | Allows inspection before automatic filesystem updates begin. |

Confirm the workspace path, branch enrollment rule, Python environment, and controller launch method before enabling updates. Other branch names can be allowlisted; no branch renaming is required by this plan.

## 3. Files to build locally

Suggested repository location: `Tools/PiAutomation/`. These files are future deliverables, not files already implemented by this plan.

| File | Responsibility |
| --- | --- |
| `sync_checkouts.py` | Discover branches, create checkouts, validate state, fast-forward eligible checkouts, and record outcomes. |
| `run_controller.py` | Launch an explicitly selected controller with the configured Python environment while holding run locks. |
| `manage_sync.py` | Show status, pause/resume updates globally or per checkout, and request a single sync pass. |
| `install_code_sync.py` | Validate configuration and install reviewed standalone scripts and user service files. Default to disabled. |
| `config.example.json` | Document the configuration schema without credentials or real machine assumptions. |
| `systemd/engt480-code-sync.service` | Run one bounded synchronization pass. |
| `systemd/engt480-code-sync.timer` | Schedule synchronization passes. |
| `Tests/test_code_sync.py` | Exercise Git behavior with disposable local repositories. |
| `Tests/test_run_locks.py` | Verify run/update exclusion and process cleanup without hardware. |
| `README.md` | Explain installation, launching, status, pauses, troubleshooting, and removal. |

Install the updater outside all synchronized checkouts. A branch update must not replace the running updater or execute a branch's installer. Updating the automation itself should be an explicit maintenance action.

An illustrative configuration should contain:

```json
{
  "remote": "git@github.com:etwan007/ENGT480.git",
  "workspace": "<PI_WORKSPACE>",
  "main_branch": "main",
  "main_directory": "ENGT480",
  "experiments_directory": "Experiments",
  "branch_patterns": ["*_EXP", "experiment/*"],
  "extra_branches": ["Ethan-wirelessTX"],
  "excluded_branches": ["main", "logs"],
  "controller_python": "<VERIFIED_VENV>/bin/python",
  "max_experiment_checkouts": 10,
  "minimum_free_bytes": 2147483648
}
```

The count and free-space thresholds are proposed starting limits. Refuse placeholder paths and validate available storage against repository size before installation. Missing allowlisted branches should be reported without being created remotely.

## 4. How a synchronization pass should work

1. Take an updater-instance lock so manual and scheduled sync passes cannot overlap. Respect persistent global and per-checkout pause settings.
2. Attempt the shared rig lock without waiting. If a managed run holds it, defer the pass. Hold it throughout Git activity so a controller cannot start partway through an update. This first version deliberately defers all sync work while the rig is in use, including fetches and new clones.
3. Discover remote branch names and commit IDs. Select only enrolled branches, and treat main separately. Exclude logs before fetching repository objects.
4. Resolve each branch to a stable directory through a registry. Simple safe names can remain unchanged. For names containing slashes or other unsupported characters, use a readable safe name plus a stable digest. Do not use raw branch names as paths. Detect name collisions and reject symlinks, path escapes, or unexpected existing directories.
5. For a new experiment, clone only its branch without tags into a temporary sibling directory. Publish the completed checkout with a same-filesystem rename only after validation. An interrupted clone must not appear as a ready experiment.
6. For an existing managed checkout, verify its canonical path, ownership, remote, expected branch, and configured fetch rules. Refuse detached HEAD, an unexpected branch, unresolved conflicts, Git operations in progress, or any staged, unstaged, or nonignored untracked changes. Ignored run logs should not block updates.
7. Fetch only that checkout's expected branch. Do not fetch all branches or tags. Compare the local commit with the fetched target. Equal means no action; a clean ancestor can fast-forward. Local commits ahead of the remote, diverged history, or a remote rewrite that is not a forward move require manual attention. Never reset to the remote.
8. Under the checkout lock, recheck the working tree immediately before applying the fast-forward. Record the old and new commits, time, branch, path, and outcome. Do not treat the rig lock as protection from arbitrary Git commands or editor saves; editors need the pause workflow described below.
9. Write status atomically. Continue with independent checkouts after a recoverable checkout-specific failure. For invalid configuration, authentication failure, or an unavailable remote, report the problem and leave checked-out files unchanged.
10. Release locks and exit. Do not delete old experiment folders or local CSVs. Retry temporary failures on a later pass.

Use argument arrays for subprocesses, noninteractive authentication, verified SSH host keys, bounded Git timeouts, and child-process-group cleanup on timeout. Never run Git with force or automatic conflict resolution. A merge attempt that fails unexpectedly should stop updates for that checkout and report its state.

Single-branch clones and narrowly configured fetches matter here: the repository's logs history contains large backups. Merely skipping a logs checkout while doing an ordinary fetch of every branch would still download that history. See [Git clone's single-branch behavior](https://git-scm.com/docs/git-clone) and [Git's fast-forward-only update behavior](https://git-scm.com/docs/git-pull).

## 5. Prevent updates during runs and editing

This is the main integration requirement. The current controller scripts do not implement a shared updater/run lock. A process-name check or a busy marker by itself cannot prevent a run from starting between an idle check and an update.

Implement a standalone launcher, installed outside the repository, that takes an exclusive lock on the common rig lock file and an exclusive lock on the selected checkout lock file before loading or launching any code from that checkout. Hold both for the entire child-process lifetime, including initialization, calibration, active control, and shutdown. The updater uses the same locks. All locks must live outside synchronized directories, use a fixed acquisition order, and remain held until the relevant child processes have exited. OS file locks are advisory, so every participating launch path must cooperate; see [Linux flock semantics](https://man7.org/linux/man-pages/man2/flock.2.html).

The launcher should:

- Run only after an operator explicitly requests a launch.
- Display the selected checkout, branch, commit, dirty status, and Python executable.
- Refuse a second managed hardware run while the rig lock is held.
- Preserve interactive input and existing controller stop behavior; forward termination signals and wait for cleanup before releasing locks.
- Support the single controller, swing-up controller, double controller, and any diagnostics or model-generation tools that must not overlap source updates. A non-hardware tool may use only its checkout lock if its implementation does not need the rig.
- Record run provenance outside the checkout without opening serial devices itself.

Directly pressing Run on a controller file in Thonny bypasses an external launcher. Before automatic mutations are enabled, either adopt the launcher as the required execution route, provide a tested Thonny-compatible route through that launcher, or implement equivalent locking at the supported entry points. Unknown or historical branches lacking that integration must not be assumed protected. If direct launches remain necessary, keep those checkouts paused until a reliable integration is in place.

Editing also needs an explicit pause: pause the checkout before working on the Pi, then save, commit/push as appropriate, close stale editor buffers, and resume. A clean Git working tree does not reveal unsaved changes in Thonny or another editor. Resuming should revalidate the checkout before any update.

## 6. Scheduler, credentials, and operating environment

Use a per-user systemd oneshot service and timer, separate from the existing CSV service. Suggested timer settings are `OnStartupSec=60s`, `OnUnitInactiveSec=60s`, and `AccuracySec=5s`. This schedules a pass roughly one minute after the preceding pass finishes. The service should have a bounded runtime, low CPU/I/O priority, and file plus journal logging. Check installed systemd compatibility during setup. See [systemd timer configuration](https://www.freedesktop.org/software/systemd/man/latest/systemd.timer.html).

Verify user lingering if updates must continue after logout and start after reboot. Prior project notes report it was enabled for the log uploader, but this plan has not rechecked the Pi. See [loginctl lingering](https://www.freedesktop.org/software/systemd/man/latest/loginctl.html).

Git, Python 3, systemd user services, network access to GitHub, and sufficient disk space are required. The updater itself should use the Python standard library. Controllers should continue using a verified environment with their existing scientific and serial packages. Do not install packages automatically on a timer or assume a virtual environment is present at a historical path.

The updater needs repository read access only. Keep its authentication separate from contributor identities used for pushing edits. The existing CSV uploader's dedicated key and service must remain configured for their existing purpose. Do not read, copy, commit, or expose private keys; do not silently change an existing checkout's remote or identity. An authentication failure should produce a pending/error status, not an interactive password prompt.

## 7. Existing Pi checkout and CSV backup integration

The current Pi filesystem, remote, and Git state are unverified in this task. Historical notes identify an ENGT481 checkout and an older source remote, while current team work uses etwan007/ENGT480. Confirm this during a later authorized setup; do not rename, repoint, overwrite, or reuse that checkout without first preserving its work and logs.

Prefer creating the new layout alongside the old checkout, then checking it before making it the normal launch location. Keep the old checkout until its source changes and data have been reconciled. If an ENGT480 destination already exists, validate it instead of replacing it.

The current uploader source accepts one repository root and scans that root's `Code/*/run_logs/**/*.csv`. It will not automatically back up logs from sibling experiment checkouts. The implementation therefore also needs:

- Multi-checkout source configuration or registry discovery for the uploader.
- Separate backup namespaces for main and each experiment, preventing equal relative filenames from colliding.
- Preservation of existing uploaded paths, original CSVs, confirmation state, compression/parts behavior, credentials, and service identity.
- Per-source handling so one active experiment's log does not unnecessarily block completed logs elsewhere.
- A check that ordinary experiment Git commits exclude generated logs, even on branches with older ignore rules.

Implement and test that extension separately from code synchronization. Do not run multiple unsynchronized uploader instances against the same cache. Do not move log directories in this first version.

### Exact upload layout and source registration

Keep one uploader service, one existing cache, the existing device ID, the existing destination repository, and the existing SSH identity. Code synchronization reads code branches; the uploader independently writes only `refs/heads/logs`. Excluding logs from code cloning does not disable or redirect log pushes.

Proposed paths on the logs branch are:

```text
<existing-device-id>/
├── Code/.../run_logs/...                       # Existing/legacy backups: preserved
└── checkouts/
    ├── main/Code/.../run_logs/...              # New ENGT480 checkout
    └── <stable-experiment-id>/Code/.../run_logs/...  # One namespace per experiment
```

The stable experiment ID comes from the checkout registry and is never reassigned to a different source. Preserve the old `Code/...` namespace for the old checkout, rather than silently treating the new main checkout as the old source. This prevents equal relative filenames in old, main, or experimental checkouts from overwriting each other's backups. A directory rename must not regenerate the Pi's device ID or the experiment's source ID.

Extend uploader configuration with a list of sources: each has a source ID, canonical checkout root, backup namespace, and enabled/retired state. Preserve the existing single-root configuration as a supported legacy input. Migrate its confirmed-file state into the legacy source while keeping its remote destination and file mapping unchanged. Store confirmations by source ID plus relative CSV path and signature; a bare relative filename is no longer a sufficient key. Use atomic state writes and keep a recoverable copy of the old state during migration.

The code-sync tool should atomically register a completed new checkout before marking it ready for normal use. The uploader discovers that registration on its next pass and scans the new source's `Code/*/run_logs/**/*.csv`. Reject duplicate registrations of the same canonical root. If a remote branch is deleted, keep its source registration so remaining local logs continue to drain. If a source directory is missing, report it; never delete its remote backups or mark pending files as uploaded.

For each source, retain open-writer deferral, the 120-second quiet period, file-change checks, lossless compression/split parts, bounded batches, and retries. Select pending files fairly across sources so a large legacy backlog does not indefinitely starve new experiments. Keep cache mutation and pushes behind the existing single-uploader lock; all sources share that serialized push path. Do not put an uploader timer inside every checkout.

### Migration and verification before changing run locations

1. Leave the existing uploader running against its existing source while new checkouts are prepared. Preserve its service/configuration names, SSH key, device ID, cache, original CSVs, and confirmed-file state; renaming the project to ENGT480 is not a reason to reset these.
2. Build and test the multi-source uploader locally. Then, only during a later authorized Pi installation, stop the uploader briefly at a safe point, save its configuration and state, install the compatible extension, atomically migrate the legacy source, and resume the same service. Leave original CSVs and existing GitHub history untouched.
3. Register the new main and experiment roots without removing the old root. Verify the uploader's status lists all intended sources and shows the exact destination namespace for each.
4. Use closed sample CSVs with the same relative name but different contents in legacy, main, and experiment sources. Confirm all are present separately in the logs branch, with byte-identical restoration. Confirm source branches, their staging areas, and controller files remain unchanged.
5. Repeat verification for retry after a failed push, interruption during state migration, active writers, missing sources, and a removed experiment branch. Restore a compressed/split backup from an experiment as well.
6. Do not make the new directories the normal run locations until their backups have been verified. Keep the legacy source registered until its backlog is confirmed and the team explicitly retires it.

Reviewing current source establishes that this migration is necessary; it does not prove that the future multi-source implementation works. Local tests of the existing uploader do not validate Pi authentication, live timer state, Linux writer detection on macOS, or the unimplemented multi-source behavior.

The new main checkout will contain only what has actually been merged to main. Do not assume changes currently on Ethan_EXP, such as the shared helpers or uploader source, are already present there. Source merging and firmware deployment are separate tasks.

## 8. Local acceptance checks before Pi installation

Use temporary local Git remotes and simulated controller processes; do not access the Pi, real serial devices, or motors.

- New enrolled branch creates exactly one complete checkout; later pushes update that same directory.
- Non-enrolled branches and logs never create experiment directories. Inspect fetch rules and object reachability to confirm logs history is not pulled into code clones.
- Main advances only with its remote main; no experiment is copied or merged automatically.
- Staged edits, unstaged edits, untracked files, local-only commits, diverged history, unexpected branches/remotes, and Git operations in progress are preserved and reported.
- Pauses survive updater restarts. Unsaved editor work is addressed by the documented pause workflow.
- A simulated run blocks updating, and an update blocks starting a simulated run. Exercise simultaneous starts, child-process signals, crashes, and cleanup.
- A deleted remote branch retains its local checkout and logs. A recreated branch with rewritten history does not overwrite that checkout.
- Branch names cannot escape the experiment root, collide silently, or redirect operations through symlinks.
- Network/authentication failures and timeouts leave usable existing checkouts; interrupted clones remain unpublished and bounded temporary state is recoverable.
- The updater never executes repository hooks, filters, tests, or scripts as an automatic deployment step. Clone/fetch/merge configuration must disable hooks and reject unapproved executable Git filters in managed repositories.
- New backup namespaces preserve equal filenames across different experiments without changing existing main backups.
- Dry-run reports discoveries and intended actions without creating checkouts or changing refs or tracked files; any operational lock/status activity must be documented.

## 9. Later installation sequence

This was the original rollout sequence. The user subsequently authorized implementation on the Pi Desktop; see the installed setup record for completed steps.

1. Finish the local tools and hardware-free tests. Review the enrollment rules and launch workflow with the team.
2. During a later authorized maintenance session, inspect the Pi's checkout state, source remote, logs, disk space, Python environment, and service capabilities.
3. Preserve existing work and establish the new ENGT480/Experiments layout without deleting the old checkout.
4. Install the reviewed automation, configuration, and service definitions with the timer disabled.
5. Run a dry-run and inspect the exact branch-to-directory mapping and any pending conflicts.
6. Complete launcher/Thonny integration and experiment-log backup coverage. Test lock exclusion with harmless processes before any real hardware run.
7. Perform one manual sync, inspect main and an experiment, then enable the timer when the installation has been accepted.

Stopping or removing code synchronization must preserve every checkout and log and leave the CSV uploader working. For source rollback, pause updates, retain the recorded prior commit, and choose an explicit reviewed rollback rather than silently resetting a working tree. No automatic hardware validation or firmware rollback is included.

## 10. Remaining decisions and completion criteria

The only setup choices requiring team input are the actual workspace location, accepted branch naming/enrollment rule, Python environment, and how operators will launch through the run lock. The proposed defaults above let implementation proceed locally without changing the Pi.

The system is ready for installation when its local tests pass, checkout creation and updates are recoverable, all supported launch paths cooperate with the locks, experiment logs have backup coverage, and the installer defaults to a reviewable disabled state. Those requirements are not fulfilled by this document alone.
