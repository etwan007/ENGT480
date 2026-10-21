"""Install reviewed Pi automation; leaves the code timer disabled by default."""
import argparse
import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys

from pi_sync import atomic_json, Synchronizer


def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def install(args):
    home = Path.home()
    if not Path('/proc').is_dir() or not shutil.which('systemctl'):
        raise RuntimeError('Install on Linux with systemd user services')
    workspace = args.workspace.expanduser().resolve()
    python = args.controller_python.expanduser().absolute()
    if not python.is_file():
        raise RuntimeError('Controller Python environment does not exist')
    upload_config = home / '.config/engt481-log-upload/config.json'
    legacy = json.loads(upload_config.read_text())
    if legacy['remote'] != args.remote:
        raise RuntimeError('Code remote and existing upload destination differ; reconcile explicitly')
    key = str(args.ssh_key.expanduser().resolve()) if args.ssh_key else legacy.get('ssh_key')
    if key and not Path(key).is_file():
        raise RuntimeError('Configured SSH key path is missing')
    state = home / '.local/state/engt480-code-sync'
    library = home / '.local/lib/engt480-code-sync'
    config_path = home / '.config/engt480-code-sync/config.json'
    if config_path.exists():
        raise RuntimeError('Already installed; review existing configuration before reinstalling')
    backup = state / 'install-backups' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    backup.mkdir(parents=True, exist_ok=False)
    units = home / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    config = dict(remote=args.remote, workspace=str(workspace), state=str(state),
                  main_branch='main', branch_patterns=['*_EXP', 'experiment/*'],
                  extra_branches=['Ethan-wirelessTX'], excluded_branches=['main', 'logs'],
                  controller_python=str(python), ssh_key=key,
                  max_experiment_checkouts=10, minimum_free_bytes=2 * 1024**3)
    workspace.mkdir(parents=True, exist_ok=True)
    library.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parent
    for name in ('pi_sync.py', 'install_code_sync.py', 'desktop_action.py'):
        shutil.copy2(source / name, library / name)
    atomic_json(state / 'registry.json', {'version': 1, 'checkouts': {}})
    atomic_json(config_path, config)
    units.joinpath('engt480-code-sync.service').write_text(f'''[Unit]
Description=Update idle ENGT480 code checkouts without changing local work

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 {quote(library / 'pi_sync.py')} --config {quote(config_path)} sync
Environment=GIT_TERMINAL_PROMPT=0
Nice=15
IOSchedulingClass=idle
TimeoutStartSec=20min
StandardOutput=append:{state / 'sync.log'}
StandardError=inherit
''')
    units.joinpath('engt480-code-sync.timer').write_text('''[Unit]
Description=Check ENGT480 code branches between runs

[Timer]
OnStartupSec=60s
OnUnitInactiveSec=60s
AccuracySec=5s
Unit=engt480-code-sync.service

[Install]
WantedBy=timers.target
''')
    for label, command in [('Run', 'run'), ('Sync Status', 'status'),
                           ('Pause Updates', 'pause'), ('Resume Updates', 'resume')]:
        desktop = workspace / ('ENGT480 ' + label + '.desktop')
        if desktop.exists():
            raise RuntimeError('Desktop launcher already exists: ' + str(desktop))
        desktop.write_text(f'''[Desktop Entry]
Type=Application
Name=ENGT480 {label}
Comment=ENGT480 code automation: {label}
Exec=/usr/bin/python3 {quote(library / 'desktop_action.py')} --config {quote(config_path)} {command}
Terminal=true
Icon=utilities-terminal
Categories=Development;
''')
        desktop.chmod(0o755)
    if args.integrate_logs:
        # The same service/cache/device/identity remain in use. Stop it before
        # replacing its standalone script and adding the atomic source registry.
        subprocess.run(['systemctl', '--user', 'stop', 'engt481-log-upload.service'], check=True)
        upload_library = home / '.local/lib/engt481-log-upload'
        shutil.copy2(upload_config, backup / 'log-upload-config.json')
        for name in ('uploaded.json',):
            original = Path(legacy['cache']) / name
            if original.exists():
                shutil.copy2(original, backup / name)
        original = upload_library / 'upload_csv_logs.py'
        shutil.copy2(original, backup / original.name)
        uploader = source.parents[1] / 'Code/Global/upload_csv_logs.py'
        if not uploader.is_file():
            raise RuntimeError('Uploader source not found alongside Tools')
        shutil.copy2(uploader, original)
        legacy['source_registry'] = str(state / 'registry.json')
        atomic_json(upload_config, legacy)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    if args.integrate_logs:
        subprocess.run(['systemctl', '--user', 'start', '--no-block', 'engt481-log-upload.service'], check=True)
    print('Installed; code timer is not enabled. Configuration:', config_path)
    print('Original log configuration/state/script backups:', backup)
    print('Dry run: /usr/bin/python3', library / 'pi_sync.py', 'sync --dry-run')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'Desktop')
    parser.add_argument('--controller-python', type=Path, default=Path.home() / 'PythonLibraries/bin/python')
    parser.add_argument('--remote', default='git@github.com:etwan007/ENGT480.git')
    parser.add_argument('--ssh-key', type=Path)
    parser.add_argument('--integrate-logs', action='store_true')
    args = parser.parse_args()
    install(args)


if __name__ == '__main__':
    main()
