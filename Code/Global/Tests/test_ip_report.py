"""Offline tests with disposable Git remotes; no Pi or GitHub access."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import install_ip_report as installer
import report_pi_ip as report


class ReportTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name).resolve()
        self.remote = self.root / 'remote.git'
        self.seed = self.root / 'source'
        self.git('init', '--bare', str(self.remote))
        self.git('init', '-b', 'main', str(self.seed))
        self.git('-C', str(self.seed), 'config', 'user.name', 'Fixture')
        self.git('-C', str(self.seed), 'config', 'user.email', 'fixture@localhost')
        (self.seed / 'Code').mkdir()
        (self.seed / 'source.py').write_text('original source\n')
        self.git('-C', str(self.seed), 'add', '.')
        self.git('-C', str(self.seed), 'commit', '-m', 'source')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'main')
        self.main = self.git('--git-dir', str(self.remote), 'rev-parse', 'main')
        self.git('-C', str(self.seed), 'switch', '--orphan', 'logs')
        (self.seed / 'existing.csv').write_bytes(b'time,angle\n0,1\n')
        self.git('-C', str(self.seed), 'add', '.')
        self.git('-C', str(self.seed), 'commit', '-m', 'existing logs')
        self.git('-C', str(self.seed), 'push', str(self.remote), 'logs')
        self.git('-C', str(self.seed), 'switch', 'main')
        self.config = dict(repo=str(self.seed), cache=str(self.root / 'cache'),
                           remote=str(self.remote), device='test-pi')
        self.reporter = report.IPReporter(**self.config)
        self.content = b'# Raspberry Pi IP address\n\n- Local IPv4: `192.168.1.42`\n- Boot ID: `boot-one`\n'
        self.discovery = patch.object(report, 'network_report', return_value=self.content).start()
        self.addCleanup(patch.stopall)

    def git(self, *args):
        return subprocess.run(['git', *args], check=True, capture_output=True, text=True).stdout.strip()

    def head(self):
        return self.git('--git-dir', str(self.remote), 'rev-parse', 'logs')

    def test_publishes_only_markdown_and_preserves_csv_source_and_confirmation(self):
        self.reporter.cache.mkdir()
        state = self.reporter.cache / 'uploaded.json'
        state.write_text('{"files": {"existing.csv": [1, 2, 3, 4]}}\n')
        before = self.head()
        self.assertEqual(self.reporter.sync(), 1)
        self.assertEqual(self.git('--git-dir', str(self.remote), 'diff', '--name-only', before, 'logs'), 'test-pi/IP.md')
        self.assertEqual(self.git('--git-dir', str(self.remote), 'show', 'logs:test-pi/IP.md'), self.content.decode().strip())
        self.assertEqual(self.git('--git-dir', str(self.remote), 'show', 'logs:existing.csv'), 'time,angle\n0,1')
        self.assertEqual(self.git('--git-dir', str(self.remote), 'rev-parse', 'main'), self.main)
        self.assertEqual(self.git('-C', str(self.seed), 'status', '--porcelain'), '')
        self.assertEqual(state.read_text(), '{"files": {"existing.csv": [1, 2, 3, 4]}}\n')
        head = self.head()
        with patch.object(self.reporter, 'git', side_effect=AssertionError('unchanged report must not use Git')):
            self.assertEqual(self.reporter.sync(), 0)
        self.assertEqual(self.head(), head)

    def test_new_boot_and_address_change_each_publish(self):
        self.reporter.sync()
        first = self.head()
        self.discovery.return_value = self.content.replace(b'boot-one', b'boot-two')
        self.reporter.sync()
        self.assertNotEqual(self.head(), first)
        second = self.head()
        self.discovery.return_value = self.discovery.return_value.replace(b'192.168.1.42', b'192.168.1.99')
        self.reporter.sync()
        self.assertNotEqual(self.head(), second)

    def test_shared_csv_lock_defers_without_confirming(self):
        self.reporter.cache.mkdir()
        before = self.head()
        with (self.reporter.cache / 'upload.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(self.reporter.sync(), 0)
        self.assertEqual(self.head(), before)
        self.assertFalse((self.reporter.cache / 'ip-report.json').exists())
        self.assertEqual(self.reporter.sync(), 1)

    def test_failed_push_retries_later_without_confirming(self):
        original = self.reporter.git
        def fail_push(*args, **kwargs):
            if args[0] == 'push':
                return subprocess.CompletedProcess(args, 1, b'', b'offline')
            return original(*args, **kwargs)
        before = self.head()
        with patch.object(self.reporter, 'git', side_effect=fail_push):
            with self.assertRaisesRegex(RuntimeError, 'next timer pass'):
                self.reporter.sync()
        self.assertEqual(self.head(), before)
        self.assertFalse((self.reporter.cache / 'ip-report.json').exists())
        self.assertNotIn('GIT_INDEX_FILE', self.reporter.environment)
        self.assertEqual(self.reporter.sync(), 1)

    def test_racing_remote_push_preserves_both_reports(self):
        peer = report.IPReporter(**dict(self.config, cache=str(self.root / 'peer-cache'), device='other-pi'))
        original = self.reporter.git
        raced = False
        def race(*args, **kwargs):
            nonlocal raced
            if args[0] == 'push' and not raced:
                raced = True
                peer.sync()
            return original(*args, **kwargs)
        with patch.object(self.reporter, 'git', side_effect=race):
            self.reporter.sync()
        for name in ('test-pi', 'other-pi'):
            self.assertEqual(self.git('--git-dir', str(self.remote), 'show', f'logs:{name}/IP.md'), self.content.decode().strip())
        self.assertEqual(self.git('--git-dir', str(self.remote), 'show', 'logs:existing.csv'), 'time,angle\n0,1')

    def test_missing_logs_branch_is_created_without_changing_main(self):
        self.git('--git-dir', str(self.remote), 'update-ref', '-d', 'refs/heads/logs')
        self.reporter.sync()
        self.assertEqual(self.git('--git-dir', str(self.remote), 'ls-tree', '--name-only', '-r', 'logs'), 'test-pi/IP.md')
        self.assertEqual(self.git('--git-dir', str(self.remote), 'rev-parse', 'main'), self.main)

    def test_interrupted_local_confirmation_does_not_duplicate_commit(self):
        self.reporter.sync()
        before = self.head()
        (self.reporter.cache / 'ip-report.json').unlink()
        self.reporter.sync()
        self.assertEqual(self.head(), before)

    def test_network_not_ready_leaves_remote_unchanged(self):
        before = self.head()
        self.discovery.side_effect = RuntimeError('No route yet')
        with self.assertRaisesRegex(RuntimeError, 'No route'):
            self.reporter.sync()
        self.assertEqual(self.head(), before)
        self.assertFalse(self.reporter.cache.exists())

    def test_logs_only_download_loads_existing_installed_uploader(self):
        home = self.root / 'isolated-home'
        library = home / '.local/lib/engt481-log-upload'
        library.mkdir(parents=True)
        source = Path(report.__file__).parent
        (library / 'upload_csv_logs.py').write_bytes((source / 'upload_csv_logs.py').read_bytes())
        download = self.root / 'logs-download'
        download.mkdir()
        for name in ('install_ip_report.py', 'report_pi_ip.py'):
            (download / name).write_bytes((source / name).read_bytes())
        env = dict(os.environ, HOME=str(home), PYTHONDONTWRITEBYTECODE='1')
        env.pop('PYTHONPATH', None)
        for name in ('install_ip_report.py', 'report_pi_ip.py'):
            result = subprocess.run([sys.executable, str(download / name), '--help'],
                                    cwd=download, env=env, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--config', result.stdout)
        (library / 'upload_csv_logs.py').unlink()
        result = subprocess.run([sys.executable, str(download / 'install_ip_report.py'), '--help'],
                                cwd=download, env=env, capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Install the existing CSV uploader', result.stderr)

    def test_installer_requires_linger_and_only_enables_ip_units(self):
        home = self.root / 'home'
        library = home / '.local/lib/engt481-log-upload'
        library.mkdir(parents=True)
        existing = library / 'upload_csv_logs.py'
        existing.write_bytes(b'existing installed uploader\n')
        config_file = home / 'upload-config.json'
        config_file.write_text(json.dumps(self.config))
        with patch.object(installer.Path, 'home', return_value=home), \
                patch.object(installer.sys, 'platform', 'linux'), \
                patch.object(installer.shutil, 'which', return_value='/usr/bin/tool'), \
                patch.object(installer.subprocess, 'run') as run:
            run.return_value = subprocess.CompletedProcess([], 0, 'no\n', '')
            with self.assertRaisesRegex(RuntimeError, 'enable-linger'):
                installer.install(config_file)
            self.assertFalse((home / '.config/systemd/user').exists())
            run.return_value = subprocess.CompletedProcess([], 0, 'yes\n', '')
            installer.install(config_file)
            units = home / '.config/systemd/user'
            self.assertIn('OnBootSec=30s', (units / 'engt480-ip-report.timer').read_text())
            self.assertIn('OnUnitInactiveSec=60s', (units / 'engt480-ip-report.timer').read_text())
            self.assertIn(str(config_file), (units / 'engt480-ip-report.service').read_text())
            self.assertEqual(existing.read_bytes(), b'existing installed uploader\n')
            self.assertEqual(json.loads(config_file.read_text()), self.config)
            self.assertTrue((library / 'report_pi_ip.py').is_file())
            systemctl_calls = [call.args[0] for call in run.call_args_list if call.args[0][0] == 'systemctl']
            self.assertTrue(any('enable' in cmd and 'engt480-ip-report.timer' in cmd for cmd in systemctl_calls))
            self.assertTrue(all(not any('log-upload.' in arg for arg in cmd) for cmd in systemctl_calls))


class DiscoveryTests(unittest.TestCase):
    def test_route_source_is_used_and_loopback_link_local_are_rejected(self):
        # Exercise actual discovery separately from the Git integration fixture.
        for address, valid in [('192.168.1.42', True), ('127.0.0.1', False), ('169.254.1.2', False)]:
            result = subprocess.CompletedProcess([], 0, json.dumps([dict(prefsrc=address, dev='wlan0')]), '')
            with patch.object(report.subprocess, 'run', return_value=result), \
                    patch.object(report.Path, 'read_text', return_value='boot-test\n'), \
                    patch.object(report.socket, 'gethostname', return_value='raspberrypi'):
                if valid:
                    content = report.network_report().decode()
                    self.assertIn('Local IPv4: `192.168.1.42`', content)
                    self.assertIn('Interface: `wlan0`', content)
                    self.assertIn('Boot ID: `boot-test`', content)
                else:
                    with self.assertRaisesRegex(RuntimeError, 'No usable'):
                        report.network_report()


if __name__ == '__main__':
    unittest.main()
