# ENGT480 repository and Pi workflow — reference for AI assistants

## Purpose and current status

The team will use one GitHub repository with a main Pi checkout and separate experiment checkouts. This lets multiple people develop and test changes while keeping an approved version available.

**This document describes the intended setup. Branch discovery and automatic Pi updates have not yet been implemented.** ENGT480 is the intended project and main checkout name; do not assume existing directories have already been renamed.

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

The checkouts may be implemented using separate clones or Git worktrees. That implementation choice has not been finalized.

## What branches and folders mean

A branch represents a version of the whole repository. It does not apply only to one folder.

For example, the `Ethan-wirelessTX` branch can modify transmitter code under `Code/Wireless/` and shared helpers under `Code/Global/`. Its Pi checkout contains both changes together.

Each experiment uses the shared helpers from its own checkout. It should not import helpers from the main checkout, because that would mix versions.

The Pi’s outer `Experiments/` directory is workspace organization. It is not a directory that contributors need to add to the GitHub repository.

## Contributor workflow

1. Start from an up-to-date `main` and create a descriptive task branch.
2. Modify existing files in their normal locations, adding new scripts where appropriate.
3. Commit and push the branch to GitHub. A branch existing only on a laptop is not available to the Pi.
4. The planned automation detects an eligible experiment branch and creates its Pi checkout.
5. Future pushes update that same checkout. They do not create another checkout for each commit.
6. Run hardware experiments explicitly from the intended checkout.
7. If editing directly on the Pi, commit and push those changes before continuing work elsewhere.
8. Open a pull request into `main` when the change is ready. Review and validate the changes before merging.
9. After merging, the main Pi checkout updates at an eligible time.

Merging changes the tracked files in `main`; no manual copying from the experiment checkout is required. Review new experimental scripts before merging, because added files in the branch are included in the pull request too.

## Requirements for the planned updater

- Verify the intended GitHub remote before configuring existing Pi checkouts.
- Create checkouts only for eligible experiment branches. The eligibility rule still needs to be chosen.
- Exclude `main`, `logs`, and unrelated branches from experiment discovery.
- Map each eligible branch to one unique, safe directory name.
- Update existing checkouts using fast-forward-only pulls.
- Leave updates pending when a checkout has uncommitted changes, has diverged, or is running a controller.
- Report update conflicts instead of resetting, overwriting, or automatically committing local work.
- Never launch controllers, flash firmware, or command hardware as part of synchronization.
- Do not automatically delete experiment folders when branches are merged or deleted; they may contain local work or run data.

Automatic updates require a reliable way to identify active runs and coordinate updates with controller startup.

## Shared hardware and run data

Separate checkouts isolate source files. They still share the Pi’s motors, serial devices, and other physical hardware. Only one operator should control the rig at a time.

Record the code commit and relevant firmware version for important runs. Updating Python code does not update firmware already flashed onto a device.

Preserve the existing CSV backup workflow and `logs` branch. If experiment checkouts generate logs in new locations, explicitly configure backup coverage for those locations.
