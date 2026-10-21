# ENGT480 repository and Pi workflow — reference for AI assistants

## Purpose and current status

The team will use one GitHub repository with a main Pi checkout and separate experiment checkouts. This lets multiple people develop and test changes while keeping an approved version available.

The implementation is in [Tools/PiAutomation](../Tools/PiAutomation/README.md). The [installed Desktop setup](PI_AUTOMATION_SETUP.md) records the actual paths and verification. ENGT480 is the main checkout name; the older ENGT481 checkout is preserved separately.

For the proposed tools, update algorithm, run locks, installation requirements, and local acceptance checks, see the [Pi automation implementation plan](PI_AUTOMATION_PLAN.md).

## Directory layout

`<PI_WORKSPACE>` represents the parent directory chosen during setup. It is not a literal path.

```text
<PI_WORKSPACE>/
├── ENGT480/                         # Checkout following main
│   └── Code/
│       ├── Global/
│       ├── SinglePendulum/
│       ├── DoublePendulum/
│       ├── Wireless/
│       └── RunReview/
│
└── Experiments/
    └── Ethan-wirelessTX/             # Checkout following Ethan-wirelessTX
        └── Code/
            ├── Global/
            ├── SinglePendulum/
            ├── DoublePendulum/
            ├── Wireless/
            └── RunReview/
```

`ENGT480/` and `Experiments/` are sibling directories. Each experiment directory contains a complete checkout of the same repository.

The implementation uses independent single-branch clones with narrow fetch rules. Source clones do not fetch the logs branch; the uploader maintains its own separate Git cache.

## What branches and folders mean

A branch represents a version of the whole repository. It does not apply only to one folder.

For example, the `Ethan-wirelessTX` branch can modify transmitter code under `Code/Wireless/` and shared helpers under `Code/Global/`. Its Pi checkout contains both changes together.

Each experiment uses the shared helpers from its own checkout. It should not import helpers from the main checkout, because that would mix versions.

The Pi’s outer `Experiments/` directory is workspace organization. It is not a directory that contributors need to add to the GitHub repository.

## Contributor workflow

1. Start from an up-to-date `main` and create a descriptive task branch.
2. Modify existing files in their normal locations, adding new scripts where appropriate.
3. Commit and push the branch to GitHub. A branch existing only on a laptop is not available to the Pi.
4. The automation enrolls branches ending in `_EXP` or starting with `experiment/`, plus explicitly allowlisted names such as `Ethan-wirelessTX`, and creates their Pi checkouts.
5. Future pushes update that same checkout. They do not create another checkout for each commit.
6. Use the ENGT480 Run Desktop shortcut to explicitly select the intended checkout and script. Pause updates before Pi editing or direct Thonny use. Updates defer while Thonny is open; direct arbitrary launches cannot replace the shared launcher lock.
7. If editing directly on the Pi, commit and push those changes before continuing work elsewhere.
8. Open a pull request into `main` when the change is ready. Review and validate the changes before merging.
9. After merging, the main Pi checkout updates at an eligible time.

Merging changes the tracked files in `main`; no manual copying from the experiment checkout is required. Review new experimental scripts before merging, because added files in the branch are included in the pull request too.

## Updater behavior

- Verify the intended GitHub remote before configuring existing Pi checkouts.
- Create checkouts only for configured experiment patterns (`*_EXP`, `experiment/*`) and allowlisted names. Configuration is machine-local, outside branch checkouts.
- Exclude `main`, `logs`, and unrelated branches from experiment discovery.
- Map each eligible branch to one unique, safe directory name.
- Update existing checkouts using fast-forward-only pulls.
- Leave updates pending when a checkout has uncommitted changes, has diverged, or is running a controller.
- Report update conflicts instead of resetting, overwriting, or automatically committing local work.
- Never launch controllers, flash firmware, or command hardware as part of synchronization.
- Do not automatically delete experiment folders when branches are merged or deleted; they may contain local work or run data.

The installed standalone launcher and updater share exclusive rig/checkout locks. The updater also defers while Thonny is open or a same-user process has a recognized motor/encoder serial port open. Pause before editing; unsaved editor buffers are not visible to Git.

## Shared hardware and run data

Separate checkouts isolate source files. They still share the Pi’s motors, serial devices, and other physical hardware. Only one operator should control the rig at a time.

Record the code commit and relevant firmware version for important runs. Updating Python code does not update firmware already flashed onto a device.

The existing CSV service reads the atomic checkout registry and backs up legacy, main, and experiment roots to separate namespaces on the logs branch. Main uses `<device>/checkouts/main/Code/...`; experiments use `<device>/checkouts/<checkout-id>/Code/...`; legacy paths remain `<device>/Code/...`. Keep the service, cache, key, device ID, and legacy confirmation state. Retained experiment folders remain backup sources after their remote branch is deleted.
