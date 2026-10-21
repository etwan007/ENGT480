# ENGT480 Desktop automation

The Pi uses `~/Desktop/ENGT480` for main and `~/Desktop/Experiments/<branch>`
for opted-in experiment branches. The old checkout is preserved separately.
Code synchronization never launches controllers, flashes firmware, commits,
pushes, or overwrites local edits.

## Everyday use

- Push a branch ending in `_EXP` or starting with `experiment/`. It appears
  on the Pi during the next eligible update. `Ethan-wirelessTX` is also
  allowlisted. Other names can be added to `extra_branches` in configuration.
- Simple branch names become the same folder name. Other names receive a
  readable, stable folder name with a digest to prevent collisions.
- Use the **ENGT480 Run** Desktop shortcut to select a checkout and script.
  It uses `~/PythonLibraries/bin/python`, holds the rig/update locks throughout
  initialization, control, and shutdown, and records branch/commit provenance.
  It never launches a program without your selection. Only one managed run
  can use the rig at a time.
- Use **ENGT480 Pause Updates** before editing on the Pi. Save and commit/push
  your changes, close stale editor buffers, then use **ENGT480 Resume Updates**.
  The updater leaves dirty, locally advanced, diverged, or switched checkouts
  alone. Configure your own Git author and contributor push authentication
  before publishing Pi edits; the updater itself never publishes them.
- Prefer the Run shortcut for hardware. If using Thonny directly, pause first.
  As an additional guard, synchronization defers while Thonny is open or a
  same-user process has an encoder/motor serial port open. These checks cannot
  replace the cooperative run lock for arbitrary direct launch methods.
- Merging into main makes the main checkout eligible for its next update.
  No folder is deleted automatically when a branch is removed.

The updater intentionally skips all Git activity during a managed run. A network
operation may briefly hold the rig lock; retry a launch after it completes.

## Commands

```sh
python3 ~/.local/lib/engt480-code-sync/pi_sync.py status
python3 ~/.local/lib/engt480-code-sync/pi_sync.py sync --dry-run
python3 ~/.local/lib/engt480-code-sync/pi_sync.py sync
python3 ~/.local/lib/engt480-code-sync/pi_sync.py pause
python3 ~/.local/lib/engt480-code-sync/pi_sync.py resume
python3 ~/.local/lib/engt480-code-sync/pi_sync.py pause Ethan_EXP
python3 ~/.local/lib/engt480-code-sync/pi_sync.py resume Ethan_EXP
python3 ~/.local/lib/engt480-code-sync/pi_sync.py run
systemctl --user status engt480-code-sync.timer
tail -n 40 ~/.local/state/engt480-code-sync/sync.log
```

A dry run checks branch discovery and existing checkout state. It does not fetch,
create checkouts, or update registry/refs; operational directories/locks may be
created. On errors, existing controller files and CSVs are retained. Fix the
reported local state manually, then resume. There is no automatic reset/rebase.

## Commit and push Pi edits

The Pi's persistent `engt` alias calls the installed tool. From inside your
experiment checkout, save files and use:

```sh
cd ~/Desktop/Experiments/Ethan_EXP
engt publish "Tune transmitter timing"
```

It shows saved changes and any unpublished commits, asks for confirmation,
stages nonignored source changes, commits with your Git author, and pushes to
that experiment's existing branch. Generated `run_logs`, Python caches, and
bytecode are excluded; already staged generated files must be unstaged first.
The automatic updater still only downloads code. CSV backups still push only
logs through their separate service.

From another directory, select the checkout explicitly:

```sh
engt publish "Tune transmitter timing" --checkout Ethan_EXP
```

Publishing holds update/run locks and preserves your global/per-checkout pause
settings. It refuses main/logs, unexpected checkouts, and remote history that
needs reconciliation; it never pulls, rebases, or force-pushes. If a push fails,
the commit remains local; repeating the command retries it without creating an
extra empty commit. No source edits or unpublished commits means no commit/push.
`--yes` skips the changes confirmation when you deliberately want that behavior.

The command uses the configured repository SSH key for the explicit push,
while taking author name/email from the checkout's effective Git configuration.
Set your own author in that checkout before publishing if necessary:

```sh
git config --local user.name "Your Name"
git config --local user.email "your-email@example.com"
```

To load the alias in an older terminal session, use `source ~/.bashrc`.

## CSV backups

One existing uploader continues to push only to the `logs` branch using the
same service, key, device ID, cache, confirmation state, and remote. It scans
the legacy checkout plus checkouts registered by code sync, including retained
experiments whose remote branch was deleted. CSVs remain in their own run folders.

Existing backups remain at `<device>/Code/...`. New main backups use
`<device>/checkouts/main/Code/...`; experiment backups use
`<device>/checkouts/<checkout-id>/Code/...`. Equal filenames in different
checkouts therefore remain separate. The confirmation-state schema remains
backward compatible: old relative keys are untouched and new keys include
their stable namespace. All sources share the existing upload lock.

Active writers and recently modified CSVs are deferred per source. Batches
rotate between sources, and retain compression/splitting, verification,
idempotent retries, and explicit non-force pushes to `refs/heads/logs`.
An upload already in progress may overlap a later run, as in the previous
uploader; it runs at low scheduling priority. No Git operation runs in a
controller's working tree to upload CSVs.

```sh
systemctl --user status engt481-log-upload.timer
tail -n 30 ~/.local/state/engt481-log-upload/upload.log
```

The uploader/service's existing `engt481` names are retained for compatibility.
Missing sources are reported and their backups are retained. Stop code sync
without stopping CSV backups:

```sh
systemctl --user disable --now engt480-code-sync.timer
systemctl --user stop engt480-code-sync.service
```

## Installation and checks

From a reviewed checkout on the Pi:

```sh
python3 -m unittest discover -s Tools/PiAutomation/Tests -p 'test_*.py'
python3 -m unittest discover -s Code/Global/Tests -p 'test_log_upload.py'
python3 Tools/PiAutomation/install_code_sync.py --integrate-logs
python3 ~/.local/lib/engt480-code-sync/pi_sync.py sync --dry-run
python3 ~/.local/lib/engt480-code-sync/pi_sync.py sync
```

Installation leaves the code timer disabled. Verify registered checkout states,
safe launcher behavior with harmless scripts, and restored CSV bytes from all
three source classes before enabling:

```sh
systemctl --user enable --now engt480-code-sync.timer
```

The installer preserves old uploader configuration, confirmation state, and
script under `~/.local/state/engt480-code-sync/install-backups/`. It does not
move the old checkout or log files, change its remote, or merge branches.
Git clones fetch one branch and no tags, keeping the logs history outside
code clones. The updater is installed separately and does not update itself
from experiment scripts.
