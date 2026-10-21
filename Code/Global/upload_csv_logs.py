"""Copy closed Pi run CSVs to a logs-only Git branch, outside the controller.

Uses only the Python standard library and Git. Run using the installed user timer.
The source checkout, its index, and original CSV files are never modified.
"""
import argparse
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time

MAX_BLOB_BYTES = 90 * 1024 * 1024
COMPRESS_AFTER_BYTES = 20 * 1024 * 1024
BATCH_BYTES = 32 * 1024 * 1024


def run_git(command, *, env, data=None, timeout=90):
    """Terminate the entire Git/SSH/pack process group if a transfer expires."""
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, env=env, start_new_session=True)
    try:
        stdout, stderr = process.communicate(data, timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
        raise RuntimeError(f'Git operation exceeded {timeout} seconds; queued files will retry')
    return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)


def signature(path):
    info = path.stat()
    return [info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns]


def logs_open_for_writing(repo):
    """Conservatively defer uploads while any same-user process writes run logs."""
    proc = Path('/proc')
    if not proc.is_dir():
        raise RuntimeError('Automatic uploading requires Linux /proc to detect active runs')
    root = str(repo.resolve() / 'Code') + '/'
    for process in proc.iterdir():
        try:
            if not process.name.isdigit() or process.stat().st_uid != os.getuid():
                continue
            for fd in (process / 'fd').iterdir():
                try:
                    target = os.readlink(fd)
                    if target.startswith(root) and '/run_logs/' in target:
                        info = (process / 'fdinfo' / fd.name).read_text()
                        flags = next(line.split()[1] for line in info.splitlines()
                                     if line.startswith('flags:'))
                        if int(flags, 8) & os.O_ACCMODE != os.O_RDONLY:
                            return True
                except FileNotFoundError:
                    continue  # The process closed this descriptor during inspection.
        except FileNotFoundError:
            continue
        except PermissionError:
            # Pi user-session managers and SSH agents deliberately hide their
            # descriptors; they do not write controller CSVs. Unknown protected
            # processes still defer the upload rather than guessing they are idle.
            try:
                if (process / 'comm').read_text().strip() in {'systemd', '(sd-pam)', 'ssh-agent'}:
                    continue
            except OSError:
                pass
            return True
    return False


class LogUploader:
    def __init__(self, repo, cache, remote, device, idle_seconds=120, ssh_key=None):
        self.repo = Path(repo).resolve()
        self.cache = Path(cache).resolve()
        self.remote = remote
        if not re.fullmatch(r'[A-Za-z0-9_-]+', device):
            raise ValueError('Device name must contain only letters, digits, _ or -')
        self.device = device
        self.idle_seconds = idle_seconds
        self.git_dir = self.cache / 'logs.git'
        self.environment = dict(os.environ, GIT_TERMINAL_PROMPT='0',
                                GIT_AUTHOR_NAME='ENGT481 CSV Logger',
                                GIT_AUTHOR_EMAIL='logs@localhost',
                                GIT_COMMITTER_NAME='ENGT481 CSV Logger',
                                GIT_COMMITTER_EMAIL='logs@localhost')
        for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'):
            self.environment.pop(key, None)
        self.environment.setdefault('GIT_SSH_COMMAND', 'ssh -o BatchMode=yes -o ConnectTimeout=10')
        if ssh_key:
            self.environment['GIT_SSH_COMMAND'] = (
                'ssh -o BatchMode=yes -o ConnectTimeout=10 -o IdentitiesOnly=yes -i ' +
                shlex.quote(str(Path(ssh_key).expanduser().resolve())))

    def git(self, *args, data=None, check=True):
        result = run_git(['git', '--git-dir', str(self.git_dir), *args], data=data,
                         env=self.environment, timeout=900 if args[0] == 'push' else 90)
        if check and result.returncode:
            raise RuntimeError(result.stderr.decode(errors='replace').strip())
        return result

    def backup_blobs(self, path, name):
        """Represent large CSVs losslessly using gzip, then bounded parts if needed."""
        if path.stat().st_size <= COMPRESS_AFTER_BYTES:
            blob = self.git('hash-object', '-w', '--', str(path)).stdout.decode().strip()
            return {name: (blob, path.stat().st_size)}
        with tempfile.TemporaryDirectory(dir=self.cache) as temp:
            archive = Path(temp) / 'backup.gz'
            with path.open('rb') as source, archive.open('wb') as dest:
                with gzip.GzipFile(filename='', fileobj=dest, mode='wb', mtime=0, compresslevel=6) as zipped:
                    shutil.copyfileobj(source, zipped, length=1024 * 1024)
            size = archive.stat().st_size
            if size <= min(BATCH_BYTES, MAX_BLOB_BYTES):
                blob = self.git('hash-object', '-w', '--', str(archive)).stdout.decode().strip()
                print(f'Compressed {name}: {path.stat().st_size / 2**20:.1f} -> {size / 2**20:.1f} MiB', flush=True)
                return {name + '.gz': (blob, size)}
            # Exceptionally large archives are split into byte-exact parts. The
            # manifest is uploaded last, after every required part is confirmed.
            blobs, parts = {}, []
            digest = hashlib.sha256()
            folder = name + '.gz.parts/'
            with archive.open('rb') as source:
                while data := source.read(min(BATCH_BYTES, MAX_BLOB_BYTES)):
                    part = f'part-{len(parts):06d}'
                    digest.update(data)
                    parts.append({'name': part, 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)})
                    blob = self.git('hash-object', '-w', '--stdin', data=data).stdout.decode().strip()
                    blobs[folder + part] = (blob, len(data))
            manifest = json.dumps({'filename': path.name, 'encoding': 'gzip',
                                   'archive_sha256': digest.hexdigest(), 'parts': parts}, indent=2).encode() + b'\n'
            blob = self.git('hash-object', '-w', '--stdin', data=manifest).stdout.decode().strip()
            blobs[folder + 'manifest.json'] = (blob, len(manifest))
            return blobs

    def collect(self, uploaded):
        snapshots = {}
        batch_size = 0
        paths = sorted((p for p in (self.repo / 'Code').glob('*/run_logs/**/*.csv')
                        if p.is_file() and not p.is_symlink()),
                       key=lambda p: (p.stat().st_size <= MAX_BLOB_BYTES, str(p)))
        for path in paths:
            if path.is_symlink() or not path.is_file():
                continue
            try:
                path.resolve().relative_to(self.repo / 'Code')
            except ValueError:
                continue
            name = path.relative_to(self.repo).as_posix()
            before = signature(path)
            if uploaded.get(name) == before:
                continue
            # Waiting for closed files to settle also captures interrupted runs
            # and multi-file system-identification CSVs without a stop footer.
            if time.time() - path.stat().st_mtime < self.idle_seconds:
                continue
            blobs = self.backup_blobs(path, name)
            if signature(path) == before:
                snapshots[name] = (before, blobs)
                batch_size += sum(size for _, size in blobs.values())
                if batch_size >= BATCH_BYTES or len(snapshots) >= 50:
                    break
        return snapshots

    def push(self, entries):
        """Rebuild against the latest remote head after a concurrent upload."""
        ref = 'refs/heads/logs'
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
            records = b''.join(
                f'100644 {blob}\t{self.device}/{name}'.encode() + b'\0'
                for name, (blob, _) in entries.items())
            self.git('update-index', '-z', '--index-info', data=records)
            tree = self.git('write-tree').stdout.decode().strip()
            if parent and tree == self.git('rev-parse', parent + '^{tree}').stdout.decode().strip():
                return  # A previous push succeeded but local bookkeeping was interrupted.
            args = ['commit-tree', tree]
            if parent:
                args.extend(['-p', parent])
            commit = self.git(*args, data=f'Upload {len(entries)} CSV backup files from {self.device}\n'.encode()).stdout.decode().strip()
            # Only this explicit destination is ever pushed. Never force-push.
            print(f'Pushing {len(entries)} backup files ({sum(size for _, size in entries.values()) / 2**20:.1f} MiB)...', flush=True)
            result = self.git('push', self.remote, f'{commit}:{ref}', check=False)
            if result.returncode == 0:
                print('Push confirmed by remote.', flush=True)
                return
        raise RuntimeError('CSV push failed; files remain local and will retry. ' +
                           result.stderr.decode(errors='replace').strip())

    def sync(self):
        self.cache.mkdir(parents=True, exist_ok=True)
        with (self.cache / 'upload.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return 0
            if logs_open_for_writing(self.repo):
                print('Run logs are open for writing; deferring upload.', flush=True)
                return 0
            if not self.git_dir.exists():
                subprocess.run(['git', 'init', '--bare', str(self.git_dir)],
                               env=self.environment, check=True, capture_output=True)
            if self.git('rev-parse', '--is-bare-repository').stdout.strip() != b'true':
                raise RuntimeError('Uploader cache must be a separate bare Git repository')
            state_file = self.cache / 'uploaded.json'
            state = json.loads(state_file.read_text()) if state_file.exists() else {}
            destination = [self.remote, self.device]
            uploaded = state.get('files', {}) if state.get('destination') == destination else {}
            snapshots = self.collect(uploaded)
            if not snapshots:
                return 0
            if logs_open_for_writing(self.repo):
                return 0
            # The temporary index belongs only to the cache, never the source repo.
            with tempfile.TemporaryDirectory(dir=self.cache) as temp:
                self.environment['GIT_INDEX_FILE'] = str(Path(temp) / 'index')
                try:
                    entries = {name: blob for _, blobs in snapshots.values() for name, blob in blobs.items()}
                    batch, size = {}, 0
                    for name, blob in entries.items():
                        if batch and size + blob[1] > BATCH_BYTES:
                            self.push(batch)
                            batch, size = {}, 0
                        batch[name] = blob
                        size += blob[1]
                    if batch:
                        self.push(batch)
                finally:
                    self.environment.pop('GIT_INDEX_FILE', None)
            uploaded.update({name: stat for name, (stat, _) in snapshots.items()})
            pending = self.cache / 'uploaded.json.tmp'
            pending.write_text(json.dumps({'destination': destination, 'files': uploaded}, indent=2) + '\n')
            pending.replace(state_file)
            print(f'Uploaded {len(snapshots)} CSV files to logs ({self.device}).', flush=True)
            return len(snapshots)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    try:
        LogUploader(**config).sync()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'CSV upload failed; local files retained: {exc}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
