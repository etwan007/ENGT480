"""Install boot-time Pi IP reporting using the existing logs uploader settings."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from report_pi_ip import IPReporter


def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%').replace('$', '$$') + '"'


def install(config_file):
    if sys.platform != 'linux' or any(not shutil.which(name) for name in ('systemctl', 'loginctl', 'ip', 'git')):
        raise RuntimeError('Install on the Pi running Linux with systemd, iproute2, and Git')
    home = Path.home()
    library = home / '.local/lib/engt481-log-upload'
    if not (library / 'upload_csv_logs.py').is_file():
        raise RuntimeError('Install the existing CSV uploader first')
    config_file = config_file.expanduser().resolve()
    config = json.loads(config_file.read_text())
    IPReporter(**config)  # Validate existing settings without running any upload.
    if config.get('ssh_key') and not Path(config['ssh_key']).expanduser().is_file():
        raise RuntimeError('Configured logs SSH key is missing')
    linger = subprocess.run(['loginctl', 'show-user', str(os.getuid()), '--property=Linger', '--value'],
                            check=True, capture_output=True, text=True, timeout=10)
    if linger.stdout.strip() != 'yes':
        raise RuntimeError('Boot without login requires lingering. Run sudo loginctl enable-linger "$USER", then rerun this installer')
    units = home / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    service = units / 'engt480-ip-report.service'
    timer = units / 'engt480-ip-report.timer'
    if timer.exists():
        subprocess.run(['systemctl', '--user', 'stop', timer.name, service.name], check=True)
    script = library / 'report_pi_ip.py'
    source = Path(__file__).with_name(script.name)
    if source.resolve() != script.resolve():
        if script.exists() and script.read_bytes() != source.read_bytes():
            shutil.copy2(script, script.with_suffix('.py.previous'))
        pending = script.with_suffix('.py.tmp')
        shutil.copy2(source, pending)
        pending.replace(script)
    service.write_text(f'''[Unit]
Description=Publish Raspberry Pi local IPv4 to the logs branch

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 -u {quote(script)} --config {quote(config_file)}
Environment=GIT_TERMINAL_PROMPT=0
Nice=15
IOSchedulingClass=idle
TimeoutStartSec=20min
''')
    timer.write_text('''[Unit]
Description=Report Pi IP at boot and retry network changes every minute

[Timer]
OnBootSec=30s
OnUnitInactiveSec=60s
AccuracySec=5s
Unit=engt480-ip-report.service

[Install]
WantedBy=timers.target
''')
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'enable', '--now', timer.name], check=True)
    subprocess.run(['systemctl', '--user', 'start', '--no-block', service.name], check=True)
    print(f'Enabled boot IP reporting: branch logs, {config["device"]}/IP.md')
    print('Inspect: journalctl --user -u engt480-ip-report.service -n 30')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path.home() / '.config/engt481-log-upload/config.json')
    args = parser.parse_args()
    try:
        install(args.config)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f'IP report installation failed: {exc}\n')


if __name__ == '__main__':
    main()
