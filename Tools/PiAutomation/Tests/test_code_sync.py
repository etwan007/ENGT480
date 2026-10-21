"""Disposable Git remotes and harmless processes; never access Pi hardware."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'pi_sync.py'
spec = importlib.util.spec_from_file_location('pi_sync', MODULE)
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


class SyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.seed = self.root / 'seed'
        self.remote = self.root / 'remote.git'
        self.git('init', '--bare', str(self.remote))
        self.git('init', '-b', 'main', str(self.seed))
        self.git('-C', str(self.seed), 'config', 'user.name', 'Fixture')
        self.git('-C', str(self.seed), 'config', 'user.email', 'fixture@localhost')
        (self.seed / 'Code/SinglePendulum').mkdir(parents=True)
        self.script = self.seed / 'Code/SinglePendulum/demo.py'
        self.script.write_text('print("harmless fixture")\n')
        self.commit('initial')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'main')
        self.git('-C', str(self.seed), 'switch', '--orphan', 'logs')
        (self.seed / 'logs-only.txt').write_text('only reachable through logs history\n')
        self.commit('logs only')
        self.logs_oid = self.git('-C', str(self.seed), 'rev-parse', 'HEAD').strip()
        self.git('-C', str(self.seed), 'push', str(self.remote), 'logs')
        self.git('-C', str(self.seed), 'switch', '-c', 'Ethan_EXP', 'main')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'Ethan_EXP')
        self.config = dict(remote=str(self.remote), workspace=str(self.root / 'Desktop'),
                           state=str(self.root / 'state'), controller_python=sys.executable,
                           branch_patterns=['*_EXP', 'experiment/*'], minimum_free_bytes=0)
        self.sync = sync.Synchronizer(self.config)
        self.addCleanup(patch.stopall)
        patch.object(sync, 'busy_reason', return_value=None).start()

    def git(self, *args, check=True):
        result = subprocess.run(['git', *args], capture_output=True, text=True,
                                env=dict(os.environ, GIT_CONFIG_GLOBAL='/dev/null', GIT_CONFIG_NOSYSTEM='1'))
        if check and result.returncode:
            self.fail(result.stderr)
        return result.stdout if check else result

    def commit(self, message):
        self.git('-C', str(self.seed), 'add', '.')
        self.git('-C', str(self.seed), 'commit', '-m', message)

    def advance(self):
        self.script.write_text(self.script.read_text() + '# next change\n')
        self.commit('advance')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'Ethan_EXP')

    @property
    def experiment(self):
        return Path(self.config['workspace']) / 'Experiments/Ethan_EXP'

    def test_creates_updates_same_checkout_and_never_fetches_logs(self):
        result = self.sync.sync()
        self.assertEqual(result['main']['status'], 'created')
        self.assertEqual(result['Ethan_EXP']['status'], 'created')
        self.assertNotIn('logs', self.sync.registry()['checkouts'])
        self.assertNotEqual(self.git('-C', str(self.experiment), 'cat-file', '-e', self.logs_oid, check=False).returncode, 0)
        self.advance()
        result = self.sync.sync()
        self.assertEqual(result['Ethan_EXP']['status'], 'updated')
        self.assertEqual((self.experiment / 'Code/SinglePendulum/demo.py').read_text(), self.script.read_text())
        self.assertEqual(len(list(self.experiment.parent.iterdir())), 1)

    def test_dirty_and_untracked_staged_and_local_commits_are_preserved(self):
        self.sync.sync()
        local = self.experiment / 'Code/SinglePendulum/demo.py'
        local.write_text('local work\n')
        self.advance()
        self.assertEqual(self.sync.sync()['Ethan_EXP']['status'], 'attention')
        self.assertEqual(local.read_text(), 'local work\n')
        self.git('-C', str(self.experiment), 'add', '.')
        self.assertEqual(self.sync.sync()['Ethan_EXP']['status'], 'attention')
        self.git('-C', str(self.experiment), '-c', 'user.name=Fixture', '-c', 'user.email=f@localhost', 'commit', '-m', 'local')
        self.assertEqual(self.sync.sync()['Ethan_EXP']['status'], 'attention')
        self.assertEqual(local.read_text(), 'local work\n')

    def test_deleted_branch_retains_folder_data_and_registration(self):
        self.sync.sync()
        log = self.experiment / 'Code/SinglePendulum/run_logs/retained.csv'
        log.parent.mkdir(parents=True)
        log.write_text('retain\n')
        self.git('-C', str(self.seed), 'push', str(self.remote), '--delete', 'Ethan_EXP')
        self.sync.sync()
        self.assertEqual(log.read_text(), 'retain\n')
        self.assertFalse(self.sync.registry()['checkouts']['Ethan_EXP']['active'])

    def test_dry_run_pause_and_lock_do_not_mutate_checkouts(self):
        self.assertEqual(self.sync.sync(True)['main']['status'], 'would-create')
        self.assertFalse(Path(self.config['workspace']).exists())
        self.assertFalse(self.sync.registry_file.exists())
        (self.sync.state / 'paused').touch()
        self.assertIn('deferred', self.sync.sync())
        (self.sync.state / 'paused').unlink()
        with sync.lock(self.sync.state / 'rig.lock'):
            with self.assertRaises(BlockingIOError):
                self.sync.sync()
        self.assertFalse(self.experiment.exists())

    def test_existing_destination_and_symlink_are_not_adopted(self):
        self.experiment.parent.mkdir(parents=True)
        self.experiment.symlink_to(self.seed, target_is_directory=True)
        result = self.sync.sync()
        self.assertEqual(result['Ethan_EXP']['status'], 'attention')
        self.assertTrue(self.experiment.is_symlink())

    def test_branch_names_safe_mapping_and_unrelated_branch_ignored(self):
        self.git('-C', str(self.seed), 'push', str(self.remote), 'main:refs/heads/experiment/one')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'main:refs/heads/unrelated')
        result = self.sync.sync()
        name = sync.safe_name('experiment/one')
        self.assertEqual(result[name]['status'], 'created')
        self.assertNotIn('unrelated', result)
        self.assertNotIn('/', name)
        self.assertNotEqual(sync.safe_name('experiment/one'), sync.safe_name('experiment-one'))

    def test_thonny_or_serial_guard_defers_every_git_operation(self):
        with patch.object(sync, 'busy_reason', return_value='serial busy'), patch.object(self.sync, 'git') as git:
            self.assertEqual(self.sync.sync(), {'deferred': 'serial busy'})
            git.assert_not_called()

    def test_rewritten_remote_refuses_overwrite(self):
        self.sync.sync()
        head = self.git('-C', str(self.experiment), 'rev-parse', 'HEAD')
        self.git('-C', str(self.seed), 'switch', '--orphan', 'replacement')
        (self.seed / 'replacement.txt').write_text('different ancestry')
        self.commit('replacement')
        self.git('-C', str(self.seed), 'push', '--force', str(self.remote), 'replacement:Ethan_EXP')
        self.assertEqual(self.sync.sync()['Ethan_EXP']['status'], 'attention')
        self.assertEqual(head, self.git('-C', str(self.experiment), 'rev-parse', 'HEAD'))

    def test_untracked_file_and_missing_remote_preserve_local_state(self):
        self.sync.sync()
        local = self.experiment / 'notes.txt'
        local.write_text('unfinished notes')
        self.advance()
        self.assertEqual(self.sync.sync()['Ethan_EXP']['status'], 'attention')
        self.assertEqual(local.read_text(), 'unfinished notes')
        self.sync.remote = str(self.root / 'missing-remote')
        with self.assertRaises(RuntimeError):
            self.sync.sync()
        self.assertEqual(local.read_text(), 'unfinished notes')

    def test_install_preserves_legacy_config_and_leaves_code_timer_disabled(self):
        install_path = MODULE.parent / 'install_code_sync.py'
        sys.path.insert(0, str(MODULE.parent))
        install_spec = importlib.util.spec_from_file_location('installer', install_path)
        installer = importlib.util.module_from_spec(install_spec)
        install_spec.loader.exec_module(installer)
        home = self.root / 'fake-home'
        upload_library = home / '.local/lib/engt481-log-upload'
        upload_library.mkdir(parents=True)
        (upload_library / 'upload_csv_logs.py').write_text('old uploader')
        config_path = home / '.config/engt481-log-upload/config.json'
        legacy = dict(repo=str(self.seed), cache=str(home / 'cache'), remote=str(self.remote),
                      device='preserved-device', idle_seconds=120)
        sync.atomic_json(config_path, legacy)
        sync.atomic_json(home / 'cache/uploaded.json', {'destination': [str(self.remote), 'preserved-device'], 'files': {'old.csv': [1,2,3,4]}})
        args = type('Args', (), dict(workspace=home / 'Desktop', controller_python=Path(sys.executable), remote=str(self.remote), ssh_key=None, integrate_logs=True))()
        original_is_dir = Path.is_dir
        with patch.object(Path, 'home', return_value=home), patch.object(Path, 'is_dir', lambda p: True if str(p) == '/proc' else original_is_dir(p)), patch.object(installer.shutil, 'which', return_value='/usr/bin/systemctl'), patch.object(installer.subprocess, 'run') as service:
            installer.install(args)
        updated = json.loads(config_path.read_text())
        for key, value in legacy.items():
            self.assertEqual(updated[key], value)
        self.assertEqual(json.loads((home / 'cache/uploaded.json').read_text())['files'], {'old.csv': [1,2,3,4]})
        self.assertFalse(any('enable' in call.args[0] for call in service.call_args_list))
        self.assertEqual(len(list((home / 'Desktop').glob('*.desktop'))), 4)
        self.assertTrue((home / '.config/systemd/user/engt480-code-sync.timer').is_file())

    def test_launcher_holds_locks_until_child_exits_without_hardware(self):
        self.sync.sync()
        marker = self.root / 'ready'
        demo = self.experiment / 'Code/SinglePendulum/demo.py'
        demo.write_text('from pathlib import Path\nimport time\nPath(' + repr(str(marker)) + ').touch()\ntime.sleep(30)\n')
        config = self.root / 'config.json'
        config.write_text(json.dumps(self.config))
        child = subprocess.Popen([sys.executable, str(MODULE), '--config', str(config), 'run', 'Ethan_EXP', 'Code/SinglePendulum/demo.py'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic() + 8
            while not marker.exists() and time.monotonic() < deadline and child.poll() is None:
                time.sleep(.03)
            self.assertTrue(marker.exists(), 'harmless launcher fixture did not start')
            with self.assertRaises(BlockingIOError):
                self.sync.sync()
        finally:
            child.terminate()
            child.communicate(timeout=8)
        with sync.lock(self.sync.state / 'rig.lock'):
            pass


if __name__ == '__main__':
    unittest.main()
