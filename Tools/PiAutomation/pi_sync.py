"""Conservative Pi branch synchronization and explicitly requested controller runs."""
import argparse
from contextlib import contextmanager, ExitStack
import fcntl
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time

DEFAULT_CONFIG = Path.home() / '.config/engt480-code-sync/config.json'


def read_json(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).exists() else default


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


@contextmanager
def lock(path, blocking=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        yield stream.fileno()


def safe_name(branch):
    if re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', branch) and branch != 'main':
        return branch
    slug = re.sub(r'[^A-Za-z0-9_-]', '-', branch)[:55].strip('-') or 'branch'
    return slug + '--' + hashlib.sha256(branch.encode()).hexdigest()[:16]


def busy_reason():
    """Extra conservative guard; cooperative run locks remain authoritative."""
    proc = Path('/proc')
    if not proc.exists():
        return None  # Allows hardware-free tests on macOS.
    for process in proc.iterdir():
        try:
            if not process.name.isdigit() or process.stat().st_uid != os.getuid():
                continue
            argv = (process / 'cmdline').read_bytes().split(b'\0')
            if any(b'thonny' in arg.lower() for arg in argv[:4]):
                return 'Thonny is open; close it or keep updates paused while editing'
            for fd in (process / 'fd').iterdir():
                try:
                    target = os.readlink(fd)
                    if target.startswith(('/dev/ttyUSB', '/dev/ttyACM', '/dev/serial/')):
                        return 'a serial device is open (PID ' + process.name + ')'
                except FileNotFoundError:
                    pass
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return None


class Synchronizer:
    def __init__(self, config):
        self.config = config
        self.workspace = Path(config['workspace']).expanduser()
        self.state = Path(config['state']).expanduser()
        if not self.workspace.is_absolute() or '<' in str(self.workspace):
            raise ValueError('Use a real absolute workspace path')
        if self.workspace.is_symlink() or self.workspace.resolve() != self.workspace:
            raise ValueError('Workspace must not use symlink paths')
        self.remote = config['remote']
        self.registry_file = self.state / 'registry.json'
        self.state.mkdir(parents=True, exist_ok=True)
        (self.state / 'empty-hooks').mkdir(exist_ok=True)
        self.env = dict(os.environ, GIT_TERMINAL_PROMPT='0', GIT_LFS_SKIP_SMUDGE='1',
                        GIT_CONFIG_GLOBAL='/dev/null', GIT_CONFIG_NOSYSTEM='1')
        for key in list(self.env):
            if key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'):
                self.env.pop(key)
        ssh = 'ssh -o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=yes'
        if config.get('ssh_key'):
            ssh += ' -o IdentitiesOnly=yes -i ' + shlex.quote(config['ssh_key'])
        self.env['GIT_SSH_COMMAND'] = ssh

    def git(self, *args, cwd=None, check=True, timeout=90):
        command = ['git', '-c', 'core.hooksPath=' + str(self.state / 'empty-hooks'),
                   '-c', 'core.fsmonitor=false', '-c', 'submodule.recurse=false', *args]
        process = subprocess.Popen(command, cwd=cwd, env=self.env, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, start_new_session=True)
        try:
            out, err = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate()
            raise RuntimeError('Git timed out; existing work is retained')
        result = subprocess.CompletedProcess(command, process.returncode, out.decode(errors='replace'), err.decode(errors='replace'))
        if check and result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return result

    def registry(self):
        return read_json(self.registry_file, {'version': 1, 'checkouts': {}})

    def paused(self, identifier):
        return (self.state / 'paused').exists() or (self.state / 'pauses' / identifier).exists()

    def branches(self):
        result = {}
        for line in self.git('ls-remote', '--heads', self.remote).stdout.splitlines():
            oid, ref = line.split('\t', 1)
            branch = ref.removeprefix('refs/heads/')
            if branch == self.config.get('main_branch', 'main') or (
                branch not in {'main', 'logs'} | set(self.config.get('excluded_branches', [])) and (
                    branch in self.config.get('extra_branches', []) or
                    any(fnmatch.fnmatchcase(branch, p) for p in self.config.get('branch_patterns', ['*_EXP', 'experiment/*'])))):
                result[branch] = oid
        if self.config.get('main_branch', 'main') not in result:
            raise RuntimeError('Remote main branch is missing')
        return result

    def location(self, branch):
        if branch == self.config.get('main_branch', 'main'):
            return 'main', self.workspace / 'ENGT480'
        return safe_name(branch), self.workspace / 'Experiments' / safe_name(branch)

    def validate(self, path, branch, allow_dirty=False):
        if path.is_symlink() or path.resolve() != path or not path.is_dir():
            raise RuntimeError('Unexpected or symlink checkout path')
        if path.stat().st_uid != os.getuid() or not (path / '.git').is_dir():
            raise RuntimeError('Checkout ownership/type differs from managed clone')
        if self.git('rev-parse', '--show-toplevel', cwd=path).stdout.strip() != str(path):
            raise RuntimeError('Git root does not match the registered path')
        if self.git('remote', 'get-url', 'origin', cwd=path).stdout.strip() != self.remote:
            raise RuntimeError('Remote changed; reconcile manually')
        if self.git('branch', '--show-current', cwd=path).stdout.strip() != branch:
            raise RuntimeError('Branch changed or detached HEAD; reconcile manually')
        expected = f'+refs/heads/{branch}:refs/remotes/origin/{branch}'
        if self.git('config', '--get-all', 'remote.origin.fetch', cwd=path).stdout.splitlines() != [expected]:
            raise RuntimeError('Fetch rules changed; refusing broad or unexpected fetch')
        filters = self.git('config', '--get-regexp', r'^filter\..*\.(clean|smudge|process)$', cwd=path, check=False)
        if filters.returncode == 0:
            raise RuntimeError('Executable Git filters require manual review')
        for item in ['index.lock', 'HEAD.lock', 'config.lock', 'MERGE_HEAD', 'CHERRY_PICK_HEAD',
                     'REVERT_HEAD', 'rebase-merge', 'rebase-apply', 'sequencer']:
            if (path / '.git' / item).exists():
                raise RuntimeError('Another Git operation is in progress')
        if not allow_dirty and self.git('status', '--porcelain', '--untracked-files=all', cwd=path).stdout:
            raise RuntimeError('Local edits or untracked files; save/commit/push before resuming')

    def create(self, path, branch):
        if path.exists() or path.is_symlink():
            raise RuntimeError('Unregistered destination already exists; it has not been modified')
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.parent.is_symlink() or path.parent.resolve() != path.parent:
            raise RuntimeError('Checkout parent must not be a symlink')
        if shutil.disk_usage(path.parent).free < self.config.get('minimum_free_bytes', 2 * 1024**3):
            raise RuntimeError('Insufficient free space for new checkout')
        with tempfile.TemporaryDirectory(prefix='.engt480-clone-', dir=path.parent) as temporary:
            clone = Path(temporary) / 'repo'
            self.git('clone', '--no-local', '--single-branch', '--no-tags', '--branch', branch,
                     '--', self.remote, str(clone), timeout=600)
            # Machine-local ignores keep generated data out of every branch.
            with (clone / '.git/info/exclude').open('a') as stream:
                stream.write('\n__pycache__/\n*.py[cod]\n/Code/*/run_logs/\n')
            self.validate(clone, branch)
            os.rename(clone, path)

    def update(self, path, branch, identifier):
        self.validate(path, branch)
        old = self.git('rev-parse', 'HEAD', cwd=path).stdout.strip()
        self.git('fetch', '--no-tags', 'origin', f'refs/heads/{branch}', cwd=path, timeout=300)
        new = self.git('rev-parse', 'FETCH_HEAD', cwd=path).stdout.strip()
        if old == new:
            return 'current', old, new
        if self.git('merge-base', '--is-ancestor', old, new, cwd=path, check=False).returncode:
            raise RuntimeError('Local commits or remote history changed; no automatic reset')
        changed = self.git('diff', '--name-only', old, new, cwd=path).stdout.splitlines()
        if any('/run_logs/' in '/' + name for name in changed):
            raise RuntimeError('Update changes tracked run logs; review manually')
        self.validate(path, branch)
        if self.paused(identifier):
            raise RuntimeError('Checkout paused before update')
        self.git('merge', '--ff-only', new, cwd=path)
        return 'updated', old, new

    def sync(self, dry_run=False):
        with lock(self.state / 'update.lock'), lock(self.state / 'rig.lock'):
            registry = self.registry()
            if self.paused('__all__'):
                return {'deferred': 'updates paused'}
            reason = busy_reason()
            if reason:
                return {'deferred': reason}
            branches = self.branches()
            outcomes = {}
            for branch in sorted(branches, key=lambda b: b != self.config.get('main_branch', 'main')):
                identifier, path = self.location(branch)
                record = registry['checkouts'].get(identifier)
                if self.paused(identifier):
                    outcomes[identifier] = {'status': 'paused'}
                    continue
                try:
                    if record and (record['branch'] != branch or record['path'] != str(path)):
                        raise RuntimeError('Registry collision; no checkout was changed')
                    if dry_run:
                        if record:
                            self.validate(path, branch)
                        elif path.exists() or path.is_symlink():
                            raise RuntimeError('Unregistered destination exists')
                        outcomes[identifier] = {'status': 'would-check' if record else 'would-create', 'path': str(path), 'remote_commit': branches[branch]}
                        continue
                    with lock(self.state / 'locks' / (identifier + '.lock')):
                        if not record:
                            count = sum(r['branch'] != self.config.get('main_branch', 'main') for r in registry['checkouts'].values())
                            if identifier != 'main' and count >= self.config.get('max_experiment_checkouts', 10):
                                raise RuntimeError('Experiment checkout limit reached')
                            self.create(path, branch)
                            head = self.git('rev-parse', 'HEAD', cwd=path).stdout.strip()
                            record = {'id': identifier, 'branch': branch, 'path': str(path),
                                      'namespace': 'checkouts/' + identifier, 'head': head}
                            registry['checkouts'][identifier] = record
                            status, old, head = 'created', None, head
                        else:
                            status, old, head = self.update(path, branch, identifier)
                        record.update(active=True, head=head, status=status, checked_at=time.time())
                        record.pop('error', None)
                        atomic_json(self.registry_file, registry)  # Registers backup source before launch.
                        outcomes[identifier] = {'status': status, 'old': old, 'commit': head, 'path': str(path)}
                        if status != 'current':
                            with (self.state / 'history.jsonl').open('a') as stream:
                                stream.write(json.dumps(dict(time=time.time(), branch=branch, **outcomes[identifier])) + '\n')
                except (OSError, RuntimeError, ValueError) as exc:
                    outcomes[identifier] = {'status': 'attention', 'reason': str(exc)}
                    if record:
                        record.update(status='attention', error=str(exc))
            if not dry_run:
                for record in registry['checkouts'].values():
                    if record['branch'] not in branches:
                        record.update(active=False, status='remote-removed; local files retained')
                atomic_json(self.registry_file, registry)
                atomic_json(self.state / 'status.json', {'checked_at': time.time(), 'checkouts': outcomes})
            return outcomes


def choose(items, label):
    for index, (name, _) in enumerate(items, 1):
        print(f'{index}. {name}')
    try:
        selection = int(input(label + ' (number, Ctrl+C to cancel): ')) - 1
        if selection < 0:
            raise ValueError()
        return items[selection][1]
    except (IndexError, ValueError):
        raise RuntimeError('Invalid selection; nothing was launched')


def run_controller(sync, identifier=None, script=None, script_args=()):
    registry = sync.registry()['checkouts']
    if not identifier:
        identifier = choose([(r['id'] + ' [' + r['branch'] + ']', r['id']) for r in registry.values()], 'Checkout')
    if identifier not in registry:
        raise RuntimeError('Unknown checkout')
    record = registry[identifier]
    root = Path(record['path'])
    if root.is_symlink() or root.resolve() != root:
        raise RuntimeError('Checkout path changed')
    if not script:
        candidates = []
        for relative in ['Code/SinglePendulum/PythonMaster_auto.py',
                         'Code/SinglePendulum/PythonMaster_autocalibrate.py',
                         'Code/SinglePendulum/PythonMasterSwingUpExp.py',
                         'Code/DoublePendulum/PythonMaster2_auto.py',
                         'Code/Global/AngleSensorDiagnostic.py',
                         'Code/SinglePendulum/AngleSensorDiagnostic.py']:
            if (root / relative).is_file():
                candidates.append((relative, relative))
        candidates.append(('Other script (enter its path relative to this checkout)', None))
        script = choose(candidates, 'Script') or input('Relative script path: ').strip()
    path = root / script
    try:
        path.resolve().relative_to(root / 'Code')
    except ValueError:
        raise RuntimeError('Script must be inside this checkout\'s Code directory')
    if not path.is_file() or path.suffix != '.py' or path.is_symlink():
        raise RuntimeError('Choose a regular Python script')
    with lock(sync.state / 'rig.lock') as rig_fd, lock(sync.state / 'locks' / (identifier + '.lock')) as checkout_fd:
        reason = busy_reason()
        if reason:
            raise RuntimeError(reason)
        head = sync.git('rev-parse', 'HEAD', cwd=root).stdout.strip()
        branch = sync.git('branch', '--show-current', cwd=root).stdout.strip()
        if branch != record['branch']:
            raise RuntimeError('Checkout branch changed; reconcile before running')
        dirty = bool(sync.git('status', '--porcelain', cwd=root).stdout)
        print(f'Checkout: {root}\nBranch: {branch}\nCommit: {head}\nLocal edits: {dirty}\nPython: {sync.config["controller_python"]}', flush=True)
        with (sync.state / 'runs.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(time=time.time(), checkout=identifier, branch=branch, commit=head, dirty=dirty, script=script)) + '\n')
        child = subprocess.Popen([sync.config['controller_python'], str(path), *script_args],
                                 cwd=path.parent, pass_fds=(rig_fd, checkout_fd), start_new_session=True)
        previous = {}
        def forward(signum, frame):
            if child.poll() is None:
                os.killpg(child.pid, signum)
        try:
            for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous[signum] = signal.signal(signum, forward)
            return child.wait()
        finally:
            for signum, handler in previous.items():
                signal.signal(signum, handler)



def publish(sync, message, identifier=None, yes=False):
    """Explicitly stage, commit and non-force push the selected experiment."""
    if not message.strip():
        raise RuntimeError('Give a commit message, for example: engt publish "Tune TX timing"')
    registry = sync.registry()['checkouts']
    if identifier is None:
        here = Path.cwd().resolve()
        matches = [r['id'] for r in registry.values()
                   if here == Path(r['path']) or Path(r['path']) in here.parents]
        if len(matches) != 1:
            raise RuntimeError('Run inside an experiment checkout, or add --checkout Ethan_EXP')
        identifier = matches[0]
    if identifier not in registry:
        raise RuntimeError('Unknown checkout ID')
    record = registry[identifier]
    branch, root = record['branch'], Path(record['path'])
    if identifier == 'main' or branch in {'main', 'logs', sync.config.get('main_branch', 'main')}:
        raise RuntimeError('Publish from an experiment branch; merge into main through a pull request')
    with lock(sync.state / 'update.lock'), lock(sync.state / 'rig.lock'), lock(sync.state / 'locks' / (identifier + '.lock')):
        sync.validate(root, branch, allow_dirty=True)
        sync.git('fetch', '--no-tags', 'origin', 'refs/heads/' + branch, cwd=root, timeout=300)
        remote = sync.git('rev-parse', 'FETCH_HEAD', cwd=root).stdout.strip()
        head = sync.git('rev-parse', 'HEAD', cwd=root).stdout.strip()
        if sync.git('merge-base', '--is-ancestor', remote, head, cwd=root, check=False).returncode:
            raise RuntimeError('GitHub has new or diverged commits. Local work is unchanged; reconcile before publishing')
        staged = sync.git('diff', '--cached', '--name-only', '-z', cwd=root).stdout.split('\0')
        generated = lambda name: '/run_logs/' in '/' + name or '/__pycache__/' in '/' + name or name.endswith(('.pyc', '.pyo'))
        if any(generated(name) for name in staged if name):
            raise RuntimeError('Generated logs/cache files are already staged. Unstage them before publishing')
        status = sync.git('status', '--short', '--untracked-files=all', cwd=root).stdout
        ahead = sync.git('log', '--oneline', remote + '..HEAD', cwd=root).stdout
        if not status and not ahead:
            print('No changes or unpublished commits.', flush=True)
            return 0
        identity = {}
        environment = dict(os.environ)
        for field in ('user.name', 'user.email'):
            result = subprocess.run(['git', '-C', str(root), 'config', '--get', field],
                                    capture_output=True, text=True, env=environment)
            if result.returncode or not result.stdout.strip():
                raise RuntimeError('Configure this checkout\'s Git author first: git config --local ' + field + ' "your value"')
            identity[field] = result.stdout.strip()
        print('Checkout:', root, '\nBranch:', branch, '\nAuthor:', identity['user.name'], '<' + identity['user.email'] + '>', flush=True)
        if status:
            print('Saved changes (generated logs/caches are excluded):\n' + status, flush=True)
            print(sync.git('diff', '--stat', 'HEAD', cwd=root).stdout, flush=True)
        if ahead:
            print('Existing local commits that will also be pushed:\n' + ahead, flush=True)
        if not yes and input('Stage all source changes, commit, and push to ' + branch + '? [y/N] ').strip().lower() not in {'y', 'yes'}:
            print('Cancelled; files and staging area are unchanged.')
            return 0
        if status:
            sync.git('add', '--all', '--', '.', ':(glob,exclude)**/run_logs/**',
                     ':(glob,exclude)**/__pycache__/**', ':(glob,exclude)**/*.pyc',
                     ':(glob,exclude)**/*.pyo', cwd=root)
            if sync.git('diff', '--cached', '--quiet', cwd=root, check=False).returncode:
                sync.git('diff', '--cached', '--check', cwd=root)
                sync.git('-c', 'user.name=' + identity['user.name'], '-c', 'user.email=' + identity['user.email'],
                         'commit', '-m', message, cwd=root, timeout=300)
        committed = sync.git('rev-parse', 'HEAD', cwd=root).stdout.strip()
        if committed == remote:
            print('No source changes to commit or push. Generated files remain local.')
            return 0
        try:
            sync.git('push', 'origin', 'HEAD:refs/heads/' + branch, cwd=root, timeout=300)
        except RuntimeError as exc:
            raise RuntimeError('Push failed; your commit is saved locally. Retry engt publish with the same message after resolving the issue. ' + str(exc)) from exc
        print('Published ' + committed[:7] + ' to ' + branch + '. Pause settings are unchanged.', flush=True)
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('sync').add_argument('--dry-run', action='store_true')
    sub.add_parser('status')
    for command in ('pause', 'resume'):
        sub.add_parser(command).add_argument('checkout', nargs='?', default='all')
    run = sub.add_parser('run')
    run.add_argument('checkout', nargs='?')
    run.add_argument('script', nargs='?')
    run.add_argument('script_args', nargs=argparse.REMAINDER)
    publication = sub.add_parser('publish', help='Commit and push source changes from an experiment checkout')
    publication.add_argument('message')
    publication.add_argument('--checkout')
    publication.add_argument('--yes', action='store_true', help='Skip the interactive changes confirmation')
    args = parser.parse_args()
    try:
        sync = Synchronizer(read_json(args.config))
        if args.command == 'publish':
            return publish(sync, args.message, args.checkout, args.yes)
        if args.command == 'sync':
            try:
                result = sync.sync(args.dry_run)
            except BlockingIOError:
                result = {'deferred': 'a controller or another update holds the lock'}
            print(json.dumps(result, indent=2), flush=True)
            return int(any(isinstance(r, dict) and r.get('status') == 'attention' for r in result.values()))
        if args.command == 'status':
            print('Workspace:', sync.workspace)
            print('Updates paused:', sync.paused('__all__'))
            print(json.dumps(sync.registry(), indent=2))
            print('Latest pass:', json.dumps(read_json(sync.state / 'status.json', {}), indent=2))
        elif args.command in ('pause', 'resume'):
            if args.checkout != 'all' and args.checkout not in sync.registry()['checkouts']:
                raise RuntimeError('Unknown checkout ID')
            with lock(sync.state / 'update.lock', blocking=True):
                flag = sync.state / ('paused' if args.checkout == 'all' else 'pauses/' + args.checkout)
                flag.parent.mkdir(parents=True, exist_ok=True)
                if args.command == 'pause':
                    flag.touch()
                elif flag.exists():
                    flag.unlink()
            print(args.command + ': ' + args.checkout)
        else:
            return run_controller(sync, args.checkout, args.script, args.script_args)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, TypeError, EOFError) as exc:
        print('ENGT480:', str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
