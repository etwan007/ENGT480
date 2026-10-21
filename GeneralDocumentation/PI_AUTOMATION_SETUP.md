# Installed ENGT480 Desktop setup

Installed and enabled on October 21, 2026. Both code synchronization and CSV uploads are enabled user timers; user lingering is enabled so they can run after logout and reboot.

## Where to work

```text
/home/engt481/Desktop/
├── ENGT480/                     # GitHub main
├── Experiments/
│   ├── Ethan_EXP/               # GitHub Ethan_EXP
│   └── Jaren_EXP/               # GitHub Jaren_EXP
├── ENGT480 Run.desktop
├── ENGT480 Sync Status.desktop
├── ENGT480 Pause Updates.desktop
└── ENGT480 Resume Updates.desktop
```

The older checkout at `/home/engt481/Desktop/GitHub Repository/ENGT481` remains in place with its local work, original remote, and logs. All 353 original non-Git/cache files were checked against saved hashes and found unchanged after installation. No source changes from that checkout were automatically merged into the new main checkout.

The new main checkout follows the repository's actual main branch (at installation, `7ce12b426399662c0821f6d9b16522dbb4ac012d`). The automation implementation and current shared-helper work are on Ethan_EXP; installing this system did not merge Ethan_EXP into main. Each checkout uses its own complete version of the code.

## Branches and automatic updates

Push a branch ending in `_EXP` or starting with `experiment/` to `etwan007/ENGT480`. It becomes eligible for an experiment checkout automatically. `Ethan-wirelessTX` is also allowlisted for compatibility with the team-guide example. Add other exact names to `extra_branches` in the machine configuration if needed.

Simple branch names use the same folder name. Names containing slashes or unsupported characters receive a readable folder name plus a stable digest. There is one checkout per branch, reused for later pushes. The updater excludes logs and unrelated branches, fetches only each checkout's own branch without tags, and retains local folders after remote branch deletion.

The timer checks about one minute after the preceding pass finishes. Updates wait during managed runs, while Thonny is open, or while a same-user process has a recognized motor/encoder serial device open. Local edits, untracked files, switched branches, local commits ahead of GitHub, and rewritten/diverged histories require manual attention. Nothing is reset, stashed, committed, pushed, merged between branches, or deleted automatically.

## Desktop shortcuts

- **ENGT480 Run:** choose a checkout and then a controller or diagnostic. It uses `/home/engt481/PythonLibraries/bin/python` and holds the shared rig/update locks from startup through shutdown. Import checks passed for NumPy, serial, SymPy, and control.
- **ENGT480 Sync Status:** inspect registered checkouts, current status, and update errors.
- **ENGT480 Pause Updates:** pause all source updates before Pi editing or direct Thonny use.
- **ENGT480 Resume Updates:** resume after saving work, committing/pushing as appropriate, and closing stale editor buffers. An open Thonny session still defers updates.

Use the Run shortcut for hardware runs. Arbitrary direct launches do not acquire the shared launcher lock. Pause updates before using another execution route. Source editing and publication remain manual; configure the appropriate contributor Git author and GitHub push authentication before publishing Pi edits. The updater never publishes code on a contributor's behalf.

## CSV uploads

The existing `engt481-log-upload.timer` continues using the same remote, dedicated key, device ID, cache, and legacy confirmation state. It watches the old checkout and the atomic registry of Desktop checkouts. A deleted remote experiment remains a backup source while its local folder remains registered.

On the logs branch:

```text
<existing-device-id>/
├── Code/...                         # Original checkout backups
└── checkouts/
    ├── main/Code/...                # New Desktop main backups
    ├── Ethan_EXP/Code/...           # Ethan's backups
    └── Jaren_EXP/Code/...           # Jaren's backups
```

Original CSVs stay in their own `Code/<setup>/run_logs` folders. Equal filenames in different checkouts are backed up separately. Active-writer deferral, a 120-second quiet period, compression/splitting, fair bounded batches, and non-force retry behavior are preserved. One existing service and cache serialize uploads from every source.

Real GitHub uploads were verified byte-for-byte for closed synthetic CSVs from main, Ethan_EXP, and Jaren_EXP, plus an existing legacy log. The synthetic samples are named `automation_verification_20261021.csv` and identify themselves as verification records. The [verified logs commit](https://github.com/etwan007/ENGT480/commit/f8bf4a6912c886ef0d3bd472b6af3af36881a958) preserves the separate backup paths.

## Installed files and maintenance

- Code configuration: `~/.config/engt480-code-sync/config.json`.
- Reviewed installed updater/launcher: `~/.local/lib/engt480-code-sync/`.
- Registry, status, locks, history, and verification evidence: `~/.local/state/engt480-code-sync/`.
- Original uploader script/configuration/state backups: `~/.local/state/engt480-code-sync/install-backups/`.
- Existing uploader configuration/cache/service names retain `engt481` for compatibility.

The updater does not replace itself from synchronized branches. Updating installed automation is a separate reviewed maintenance operation. Re-running the CSV installer preserves the registered source list and legacy root.

```sh
python3 ~/.local/lib/engt480-code-sync/pi_sync.py status
python3 ~/.local/lib/engt480-code-sync/pi_sync.py pause Ethan_EXP
python3 ~/.local/lib/engt480-code-sync/pi_sync.py resume Ethan_EXP
systemctl --user status engt480-code-sync.timer
systemctl --user status engt481-log-upload.timer
tail -n 40 ~/.local/state/engt480-code-sync/sync.log
tail -n 40 ~/.local/state/engt481-log-upload/upload.log
```

To stop source synchronization while retaining log uploads:

```sh
systemctl --user disable --now engt480-code-sync.timer
systemctl --user stop engt480-code-sync.service
```

To enable it again:

```sh
systemctl --user enable --now engt480-code-sync.timer
```

## Verification

All 29 hardware-free checks passed on the Pi: 11 synchronization/installer/launcher checks and 18 uploader checks, including Linux active-writer detection. The actual installed updater also deferred while the shared rig lock was held; systemd unit validation passed. Both timers were verified enabled and active, all three new source checkouts were clean, and the controller Python environment imports passed.

No controller was run against hardware and no firmware was flashed. Machine-local evidence is stored in `~/.local/state/engt480-code-sync/installation-verification.json`.

See the [operating guide](../Tools/PiAutomation/README.md), [AI workflow reference](REPOSITORY_AND_PI_WORKFLOW.md), and [original implementation plan](PI_AUTOMATION_PLAN.md).
