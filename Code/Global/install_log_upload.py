"""Install a once-per-minute CSV uploader for the current Pi user (no sudo)."""
import argparse
import json
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import uuid


def quote_systemd(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path)
    parser.add_argument('--remote', default='git@github.com:etwan007/ENGT480.git')
    parser.add_argument('--device')
    parser.add_argument('--ssh-key', type=Path)
    args = parser.parse_args()
    root = Path.home() / '.config/engt481-log-upload'
    config_file = root / 'config.json'
    old = json.loads(config_file.read_text()) if config_file.exists() else {}
    default_repo = Path(old['repo']) if old.get('source_registry') else Path(__file__).resolve().parents[2]
    repo = (args.repo or default_repo).resolve()
    if old.get('source_registry') and repo != Path(old['repo']).resolve():
        parser.error('Keep the legacy root; register new checkouts through code sync instead')
    if not (repo / 'Code').is_dir():
        parser.error('Repository must contain the Code directory')
    if not Path('/proc').is_dir() or shutil.which('systemctl') is None:
        parser.error('Install on the Raspberry Pi running Linux with systemd')
    root = Path.home() / '.config/engt481-log-upload'
    root.mkdir(parents=True, exist_ok=True)
    config_file = root / 'config.json'
    device = args.device or old.get('device') or (
        re.sub(r'[^A-Za-z0-9_-]', '-', socket.gethostname()) + '-' + uuid.uuid4().hex[:8])
    if not re.fullmatch(r'[A-Za-z0-9_-]+', device):
        parser.error('Device name must contain only letters, digits, _ or -')
    config = dict(repo=str(repo), cache=str(Path.home() / '.local/state/engt481-log-upload'),
                  remote=args.remote, device=device, idle_seconds=120)
    for field in ('source_registry', 'sources'):
        if field in old:
            config[field] = old[field]
    if args.ssh_key:
        config['ssh_key'] = str(args.ssh_key.expanduser().resolve())
    elif old.get('ssh_key'):
        config['ssh_key'] = old['ssh_key']
    Path(config['cache']).mkdir(parents=True, exist_ok=True)
    config_file.write_text(json.dumps(config, indent=2) + '\n')
    library = Path.home() / '.local/lib/engt481-log-upload'
    library.mkdir(parents=True, exist_ok=True)
    script = library / 'upload_csv_logs.py'
    source = Path(__file__).with_name(script.name)
    if script.exists() and script.read_bytes() != source.read_bytes():
        shutil.copy2(script, script.with_suffix('.py.previous'))
    shutil.copy2(source, script)
    units = Path.home() / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    (units / 'engt481-log-upload.service').write_text(f'''[Unit]
Description=Upload closed ENGT481 CSV runs to the logs branch

[Service]
Type=oneshot
ExecStart={quote_systemd(sys.executable)} -u {quote_systemd(script)} --config {quote_systemd(config_file)}
Environment=GIT_TERMINAL_PROMPT=0
Nice=15
IOSchedulingClass=idle
TimeoutStartSec=30min
StandardOutput=append:{Path(config['cache']) / 'upload.log'}
StandardError=inherit
''')
    (units / 'engt481-log-upload.timer').write_text('''[Unit]
Description=Check for ENGT481 CSV logs every minute

[Timer]
OnBootSec=60s
OnUnitActiveSec=60s
AccuracySec=5s
Unit=engt481-log-upload.service

[Install]
WantedBy=timers.target
''')
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'enable', '--now', 'engt481-log-upload.timer'], check=True)
    subprocess.run(['systemctl', '--user', 'start', '--no-block', 'engt481-log-upload.service'], check=True)
    print(f'CSV upload timer enabled for {args.remote}, branch logs, device {device}.')
    print('Run systemctl --user status engt481-log-upload.timer to inspect the schedule.')
    print('Run journalctl --user -u engt481-log-upload.service -n 30 to inspect uploads.')
    print('Upload output: ' + str(Path(config['cache']) / 'upload.log'))


if __name__ == '__main__':
    main()
