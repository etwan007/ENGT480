"""Publish the Pi's routed local IPv4 to <device>/IP.md on the logs branch."""
import argparse
import fcntl
import importlib.util
import ipaddress
import json
from pathlib import Path
import socket
import subprocess
import tempfile

def load_uploader():
    """Support a logs-only download without shipping another CSV uploader copy."""
    source = Path(__file__).with_name('upload_csv_logs.py')
    if not source.is_file():
        source = Path.home() / '.local/lib/engt481-log-upload/upload_csv_logs.py'
    if not source.is_file():
        raise SystemExit('Install the existing CSV uploader before setting up the IP report')
    spec = importlib.util.spec_from_file_location('ip_report_csv_uploader', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LogUploader


LogUploader = load_uploader()


def network_report():
    # Ask the kernel which local address it would use; no packet is sent.
    result = subprocess.run(['ip', '-j', '-4', 'route', 'get', '1.1.1.1'],
                            check=True, capture_output=True, text=True, timeout=10)
    routes = json.loads(result.stdout)
    if not routes:
        raise RuntimeError('No IPv4 route yet; retry after networking starts')
    route = routes[0]
    address = ipaddress.IPv4Address(route.get('prefsrc') or route.get('src') or '')
    interface = route.get('dev', '')
    if (not interface or interface == 'lo' or address.is_loopback or
            address.is_link_local or address.is_unspecified or address.is_multicast):
        raise RuntimeError('No usable local IPv4 address yet')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not boot:
        raise RuntimeError('Cannot identify this boot')
    return (f'# Raspberry Pi IP address\n\n'
            f'- Local IPv4: `{address}`\n'
            f'- Hostname: `{socket.gethostname()}`\n'
            f'- Interface: `{interface}`\n'
            f'- Boot ID: `{boot}`\n').encode()


class IPReporter(LogUploader):
    """Share the installed uploader's SSH settings, bare cache, and upload lock."""

    def push_report(self, content):
        ref = 'refs/heads/logs'
        blob = self.git('hash-object', '-w', '--stdin', data=content).stdout.decode().strip()
        for _ in range(3):
            listing = self.git('ls-remote', '--exit-code', self.remote, ref, check=False)
            if listing.returncode == 0:
                self.git('fetch', '--no-tags', self.remote, ref)
                parent = self.git('rev-parse', 'FETCH_HEAD').stdout.decode().strip()
                self.git('read-tree', parent)
            elif listing.returncode == 2:
                parent = None
                self.git('read-tree', '--empty')
            else:
                raise RuntimeError(listing.stderr.decode(errors='replace').strip())
            record = f'100644 {blob}\t{self.device}/IP.md'.encode() + b'\0'
            self.git('update-index', '-z', '--index-info', data=record)
            tree = self.git('write-tree').stdout.decode().strip()
            if parent and tree == self.git('rev-parse', parent + '^{tree}').stdout.decode().strip():
                return  # Also handles a successful push interrupted before local confirmation.
            args = ['commit-tree', tree]
            if parent:
                args.extend(['-p', parent])
            commit = self.git(*args, data=f'Update Pi IP address for {self.device}\n'.encode()).stdout.decode().strip()
            result = self.git('push', self.remote, f'{commit}:{ref}', check=False)
            if result.returncode == 0:
                return
            # Another device may have uploaded CSVs. Fetch and rebuild without force.
        raise RuntimeError('IP push failed; next timer pass will retry. ' +
                           result.stderr.decode(errors='replace').strip())

    def sync(self):
        content = network_report()
        self.cache.mkdir(parents=True, exist_ok=True)
        with (self.cache / 'upload.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print('CSV upload in progress; IP report will retry.', flush=True)
                return 0
            state_file = self.cache / 'ip-report.json'
            confirmed = dict(remote=self.remote, device=self.device, report=content.decode())
            if state_file.exists() and json.loads(state_file.read_text()) == confirmed:
                return 0  # No Git/network traffic when this boot and address are unchanged.
            if not self.git_dir.exists():
                subprocess.run(['git', 'init', '--bare', str(self.git_dir)],
                               env=self.environment, check=True, capture_output=True)
            if self.git('rev-parse', '--is-bare-repository').stdout.strip() != b'true':
                raise RuntimeError('Uploader cache must be a separate bare Git repository')
            with tempfile.TemporaryDirectory(dir=self.cache) as temp:
                self.environment['GIT_INDEX_FILE'] = str(Path(temp) / 'index')
                try:
                    self.push_report(content)
                finally:
                    self.environment.pop('GIT_INDEX_FILE', None)
            pending = state_file.with_suffix('.json.tmp')
            pending.write_text(json.dumps(confirmed, indent=2) + '\n')
            pending.replace(state_file)
            print(f'Published logs:{self.device}/IP.md', flush=True)
            return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path.home() / '.config/engt481-log-upload/config.json')
    args = parser.parse_args()
    try:
        IPReporter(**json.loads(args.config.expanduser().read_text())).sync()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'Pi IP report deferred: {exc}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
