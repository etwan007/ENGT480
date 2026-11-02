# Pi IP report on the logs branch

The setup files live on the **logs** branch. A one-time installation on the Pi
is required; publishing these files does not install or enable the service.
The normal code updater excludes logs, so it will not install this helper.

After installation, the Pi publishes one Markdown file to **logs**:

```text
<existing-device-ID>/IP.md
```

The recorded device ID is `raspberrypi-c8674ffe`, so the expected location is
[`raspberrypi-c8674ffe/IP.md` on logs](https://github.com/etwan007/ENGT480/blob/logs/raspberrypi-c8674ffe/IP.md).
The installer reads the actual device ID from the Pi's existing uploader config.
The link will exist after the first successful report.

Example:

```markdown
# Raspberry Pi IP address

- Local IPv4: `192.168.1.42`
- Hostname: `raspberrypi`
- Interface: `wlan0`
- Boot ID: `01234567-89ab-cdef-0123-456789abcdef`
```

The address is the local IPv4 selected by the kernel's route toward the Internet,
usually Wi-Fi or Ethernet. A VPN can change that selected interface. It is not a
public-IP lookup. A local address normally requires access to the same LAN or a
routed connection to that LAN.

## Install once when Pi access is available

Download just the two setup scripts from logs into a temporary directory, then
run the installer. This leaves the Pi's code checkouts on their current branches
and avoids downloading the CSV backup history:

```sh
setup_dir=$(mktemp -d)
for file in install_ip_report.py report_pi_ip.py; do
    curl --fail --show-error --location \
        "https://raw.githubusercontent.com/etwan007/ENGT480/logs/Code/Global/$file" \
        --output "$setup_dir/$file" || exit 1
done
python3 "$setup_dir/install_ip_report.py"
```

Use the Pi user account that owns the existing CSV uploader, without `sudo`.
The scripts load the existing installed uploader as their dependency, so no
additional uploader copy needs to be downloaded. The installer requires:

```text
~/.local/lib/engt481-log-upload/upload_csv_logs.py
~/.config/engt481-log-upload/config.json
```

It uses the same repository, device ID, SSH key, bare Git cache, and upload lock.
It adds `report_pi_ip.py` beside the installed uploader and enables two new user
units: `engt480-ip-report.service` and `engt480-ip-report.timer`. It does not
replace the installed CSV uploader or change its config, confirmation state,
timer, or source checkouts.

The installer verifies user lingering so the timer can run at boot without a
login. The project records say this was already enabled. If the installer says
it is disabled, enable it once, then rerun the installer:

```sh
sudo loginctl enable-linger "$USER"
python3 "$setup_dir/install_ip_report.py"
```

See the official [systemd lingering documentation](https://www.freedesktop.org/software/systemd/man/latest/loginctl.html#enable-linger%20%5BUSER%E2%80%A6%5D).

## Behavior and verification

The timer makes its first attempt around 30 seconds after boot and checks again
60 seconds after each completed attempt. Installation also requests an immediate
first attempt. If networking, DNS, GitHub, credentials, or the CSV lock prevent an
upload, the timer retries. No report is marked confirmed until a push succeeds
or the remote already contains the exact report.

A new boot ID causes a fresh report even when the IP stays the same. An address
or interface change also refreshes it. Unchanged confirmed reports perform no
Git operations and create no extra commits. The file is the last successfully
reported address, not an online-status heartbeat.

Every commit preserves the current remote logs tree and changes only
`<device>/IP.md`. Pushes target only `refs/heads/logs` without force. If another
device pushes during an attempt, the helper fetches the new head and rebuilds
the report commit before retrying.

```sh
systemctl --user status engt480-ip-report.timer
journalctl --user -u engt480-ip-report.service -n 30 --no-pager
```

Request another check manually:

```sh
systemctl --user start engt480-ip-report.service
```

Stop IP reporting:

```sh
systemctl --user disable --now engt480-ip-report.timer
systemctl --user stop engt480-ip-report.service
```

Offline tests use temporary local Git remotes and mocked Linux network/systemd
calls. They also verify the two-script download can load the installed uploader
without an uploader alongside the downloaded scripts. They do not access the Pi
or GitHub. For tests in an existing development checkout containing the CSV
uploader source and these test files:

```sh
python3 -m unittest discover -s Code/Global/Tests -p 'test_ip_report.py' -v
```

The service still needs a real Pi installation and reboot check to verify the
network selection, SSH authentication, and systemd startup on that machine.
