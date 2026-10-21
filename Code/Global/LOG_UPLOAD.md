# Automatic Pi CSV uploads

The Pi uploader sends CSV run logs to the `logs` branch of
`etwan007/ENGT480`. It runs separately from the controllers once a minute.
It supports all `Code/<arm>/run_logs/**/*.csv` files, including nested
system-identification runs and closed files from interrupted runs. Existing
closed CSVs are included on the first upload. No controller edits are required.

Uploads are deferred while any same-user process has a run-log file open for
writing. Closed CSVs must also be unchanged for two minutes. A newly started
run can overlap an upload that has already begun; the uploader has low CPU and
disk scheduling priority. It performs no work inside the controller's loop.

Only CSV files from those folders are uploaded, under
`<device>/Code/<arm>/run_logs/...`. CSVs stay on the Pi. Main/code branches,
the controller checkout and staged changes are never modified. The branch is
created on the first successful upload with a history containing only logs.
A private local Git cache holds the upload history; pushes never use force.
Failed uploads retry on later timer runs. Uploads use batches of at most
32 MiB and each Git push has 15 minutes to finish. Each timer pass selects up
to 50 CSVs or about 32 MiB of backup data; a backlog drains over multiple passes.
Confirmed uploads are remembered, and interrupted transfers kill the full
Git/SSH process group so retries cannot leave competing background pushes.

CSVs over 20 MiB are compressed losslessly to `.csv.gz` before upload. If a
compressed archive is still too large for one batch, it is split into bounded
parts under `.csv.gz.parts/`, followed by a checksum manifest. No CSV is skipped
because of its original size. Plain small CSVs remain directly readable.
The Pi retains the original, uncompressed CSV in every case.

## Install on the Pi

Copy `upload_csv_logs.py`, `install_log_upload.py`, and `restore_csv_log.py`
into `Code/Global/`.
From the repository root, run:

```sh
python3 Code/Global/install_log_upload.py \
  --remote git@github.com:etwan007/ENGT480.git \
  --ssh-key ~/.ssh/engt481_logs_ed25519
```

This installs a standalone copy under `~/.local/lib/engt481-log-upload/`,
configuration under `~/.config/engt481-log-upload/`, and a user systemd timer.
The cache is under `~/.local/state/engt481-log-upload/`. Re-running the installer
updates the uploader and keeps the device name and selected SSH key.
Controllers do not import it and do not require systemd or Git themselves.

The selected SSH key needs write access to `etwan007/ENGT480`. A dedicated
repository deploy key lets the Pi upload without changing its existing GitHub
identity. Generate it on the Pi only if it does not already exist:

```sh
ssh-keygen -t ed25519 -f ~/.ssh/engt481_logs_ed25519 -N '' -C 'ENGT481 CSV uploader'
cat ~/.ssh/engt481_logs_ed25519.pub
```

Add the **public** `.pub` key to the repository's Settings → Deploy keys,
named `ENGT481 Pi CSV uploader`, and enable **Allow write access**. That grants
the Pi write access to this repository; GitHub deploy keys are not restricted to
one branch. The uploader itself only pushes `refs/heads/logs`. Keep the private
key on the Pi; never paste it into GitHub, chat, or a tracked repository file.

To run the timer before login and after logout, enable user lingering:

```sh
loginctl enable-linger "$USER"
```

## Check or control uploads

```sh
systemctl --user status engt481-log-upload.timer
tail -n 30 ~/.local/state/engt481-log-upload/upload.log
journalctl --user -u engt481-log-upload.service -n 30 --no-pager
systemctl --user start engt481-log-upload.service
```

The `upload.log` file is available even on Pi images without journal storage.

Stop automatic uploads (local CSV logging continues):

```sh
systemctl --user disable --now engt481-log-upload.timer
systemctl --user stop engt481-log-upload.service
```

Resume them:

```sh
systemctl --user enable --now engt481-log-upload.timer
```

No authentication credentials are stored in the script or JSON configuration.
If upload access fails, authorize the dedicated public key, then start the
service again; queued files remain local until a push succeeds.

## Hardware-free tests

```sh
python3 -m unittest discover -s Code/Global/Tests -p 'test_log_upload.py'
```

Tests use disposable local Git remotes; no GitHub or motor devices are accessed.


## Restore a downloaded compressed CSV

For a single `.csv.gz`, ordinary gzip tools can decompress it. Alternatively:

```sh
python3 Code/Global/restore_csv_log.py downloaded_run.csv.gz
```

For split backups, download the entire `.csv.gz.parts` directory, then run:

```sh
python3 Code/Global/restore_csv_log.py downloaded_run.csv.gz.parts/manifest.json
```

The helper checks all split-part checksums, verifies the archive, and restores
the original CSV bytes. It refuses to overwrite an existing file. `--output`
selects a different destination. The run visualizer opens the restored CSV.

GitHub blocks ordinary Git blobs over 100 MiB; compression/parts keep every
uploaded object below that limit without requiring Git LFS or changing source
CSV recording. See [GitHub large-file limits](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).


## Multiple Desktop checkouts

The ENGT480 code-sync installation extends the same uploader with an atomic
checkout registry. Its original `repo`, cache, device ID, SSH key, and confirmation
state are preserved. It scans the original checkout plus every registered
Desktop main/experiment checkout, including retained folders after branch deletion.

Existing backups keep `<device>/Code/...`; Desktop main uses
`<device>/checkouts/main/Code/...` and experiments use
`<device>/checkouts/<checkout-id>/Code/...`. Files with the same relative name
in different checkouts cannot overwrite each other. The original state-file
schema remains valid because new confirmation keys include their namespace.
One service and cache serialize pushes for all sources; source selection rotates
so a legacy backlog does not indefinitely block experiments. Active writers are
deferred per source, and original CSVs stay in their own checkout run folders.

The configured registry is `~/.local/state/engt480-code-sync/registry.json`.
Re-running `install_log_upload.py` preserves the registry and legacy source root.
Register new roots through code sync instead of changing the legacy `--repo`.
See the [Desktop setup](../../GeneralDocumentation/PI_AUTOMATION_SETUP.md) and
[automation operating guide](../../Tools/PiAutomation/README.md).
