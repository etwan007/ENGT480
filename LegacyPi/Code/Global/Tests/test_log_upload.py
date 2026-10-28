"""Exercise real Git push/retry behavior against temporary local bare remotes."""
import importlib.util
import gzip
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('csv_upload', Path(__file__).resolve().parents[1] / 'upload_csv_logs.py')
upload = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upload)
restore_spec = importlib.util.spec_from_file_location('restore_csv', Path(__file__).resolve().parents[1] / 'restore_csv_log.py')
restore_csv = importlib.util.module_from_spec(restore_spec)
restore_spec.loader.exec_module(restore_csv)


class LogUploadTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / 'controller checkout with spaces'
        self.repo.mkdir()
        self.remote = self.root / 'remote.git'
        self.git('init', '--bare', str(self.remote))
        self.git('init', '-b', 'main', str(self.repo))
        self.git('-C', str(self.repo), 'config', 'user.name', 'Test')
        self.git('-C', str(self.repo), 'config', 'user.email', 'test@localhost')
        (self.repo / 'controller.py').write_text('original\n')
        self.git('-C', str(self.repo), 'add', 'controller.py')
        self.git('-C', str(self.repo), 'commit', '-m', 'source')
        self.git('-C', str(self.repo), 'push', str(self.remote), 'main')
        (self.repo / 'controller.py').write_text('staged edit\n')
        self.git('-C', str(self.repo), 'add', 'controller.py')
        (self.repo / 'controller.py').write_text('unstaged edit\n')
        self.uploader = upload.LogUploader(self.repo, self.root / 'cache', str(self.remote), 'test-pi')
        self.addCleanup(patch.stopall)
        patch.object(upload, 'logs_open_for_writing', return_value=False).start()

    def git(self, *args):
        return subprocess.run(['git', *args], check=True, capture_output=True).stdout

    def log(self, name='SinglePendulum/run_logs/balance.csv', data=b'record_type,value\nstop,done\n'):
        path = self.repo / 'Code' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        os.utime(path, (time.time() - 180, time.time() - 180))
        return path

    def files(self):
        return self.git('--git-dir', str(self.remote), 'ls-tree', '-r', '--name-only', 'logs').decode().splitlines()

    def test_pushes_only_csv_preserves_source_branch_index_and_files(self):
        log = self.log()
        nested = self.log('DoublePendulum/run_logs/sysid/samples.csv', b'time,angle\n1,2\n')
        (nested.parent / 'secret.txt').write_text('must stay local')
        before_status = self.git('-C', str(self.repo), 'status', '--porcelain')
        before_index = (self.repo / '.git/index').read_bytes()
        main = self.git('--git-dir', str(self.remote), 'rev-parse', 'main')
        self.assertEqual(self.uploader.sync(), 2)
        self.assertEqual(self.files(), ['test-pi/Code/DoublePendulum/run_logs/sysid/samples.csv',
                                       'test-pi/Code/SinglePendulum/run_logs/balance.csv'])
        self.assertEqual(self.git('--git-dir', str(self.remote), 'show', 'logs:test-pi/Code/SinglePendulum/run_logs/balance.csv'), log.read_bytes())
        self.assertEqual(main, self.git('--git-dir', str(self.remote), 'rev-parse', 'main'))
        self.assertEqual(before_status, self.git('-C', str(self.repo), 'status', '--porcelain'))
        self.assertEqual(before_index, (self.repo / '.git/index').read_bytes())
        self.assertEqual(self.git('-C', str(self.repo), 'branch', '--show-current').strip(), b'main')

    def test_unchanged_logs_do_not_create_duplicate_commits(self):
        self.log()
        self.uploader.sync()
        head = self.git('--git-dir', str(self.remote), 'rev-parse', 'logs')
        self.assertEqual(self.uploader.sync(), 0)
        self.assertEqual(head, self.git('--git-dir', str(self.remote), 'rev-parse', 'logs'))

    def test_active_or_recent_logs_are_deferred(self):
        path = self.log()
        with patch.object(upload, 'logs_open_for_writing', return_value=True):
            self.assertEqual(self.uploader.sync(), 0)
        os.utime(path, None)
        self.assertEqual(self.uploader.sync(), 0)
        self.assertFalse((self.uploader.cache / 'uploaded.json').exists())

    def test_failed_push_keeps_local_files_pending_for_retry(self):
        path = self.log()
        git = self.uploader.git
        def fail(*args, **kwargs):
            if args[0] == 'push':
                return subprocess.CompletedProcess(args, 1, b'', b'network unavailable')
            return git(*args, **kwargs)
        with patch.object(self.uploader, 'git', side_effect=fail), self.assertRaises(RuntimeError):
            self.uploader.sync()
        self.assertTrue(path.exists())
        self.assertFalse((self.uploader.cache / 'uploaded.json').exists())
        self.assertEqual(self.uploader.sync(), 1)

    def test_concurrent_remote_update_is_preserved_and_retried(self):
        self.log()
        other = upload.LogUploader(self.repo, self.root / 'cache2', str(self.remote), 'other-pi')
        git = self.uploader.git
        advanced = False
        def advance(*args, **kwargs):
            nonlocal advanced
            if args[0] == 'push' and not advanced:
                advanced = True
                other.sync()
            return git(*args, **kwargs)
        with patch.object(self.uploader, 'git', side_effect=advance):
            self.assertEqual(self.uploader.sync(), 1)
        self.assertEqual(len(self.files()), 2)
        self.assertTrue(any(name.startswith('other-pi/') for name in self.files()))

    def test_symlink_outside_run_directory_is_not_uploaded(self):
        secret = self.root / 'outside.csv'
        secret.write_text('do not upload')
        target = self.repo / 'Code/SinglePendulum/run_logs'
        target.mkdir(parents=True)
        (target / 'link.csv').symlink_to(secret)
        self.assertEqual(self.uploader.sync(), 0)

    def test_lost_local_bookkeeping_does_not_duplicate_remote_commit(self):
        self.log()
        self.uploader.sync()
        head = self.git('--git-dir', str(self.remote), 'rev-parse', 'logs')
        (self.uploader.cache / 'uploaded.json').unlink()
        self.uploader.sync()
        self.assertEqual(head, self.git('--git-dir', str(self.remote), 'rev-parse', 'logs'))

    def test_large_csv_is_compressed_and_restored_without_changes(self):
        data = b'elapsed,angle\n' + b'1,0.25\n' * 2000
        original = self.log(data=data)
        with patch.object(upload, 'COMPRESS_AFTER_BYTES', 100):
            self.assertEqual(self.uploader.sync(), 1)
        name = self.files()[0]
        self.assertTrue(name.endswith('.csv.gz'))
        archive = self.root / 'download.csv.gz'
        archive.write_bytes(self.git('--git-dir', str(self.remote), 'show', 'logs:' + name))
        output = restore_csv.restore(archive)
        self.assertEqual(output.read_bytes(), data)
        self.assertEqual(original.read_bytes(), data)
        with self.assertRaises(FileExistsError):
            restore_csv.restore(archive)

    def test_incompressible_archive_parts_restore_and_detect_corruption(self):
        data = os.urandom(10000)
        self.log(data=data)
        with patch.object(upload, 'COMPRESS_AFTER_BYTES', 100), patch.object(upload, 'MAX_BLOB_BYTES', 2048), patch.object(upload, 'BATCH_BYTES', 2048):
            self.assertEqual(self.uploader.sync(), 1)
        download = self.root / 'download'
        for name in self.files():
            path = download / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(self.git('--git-dir', str(self.remote), 'show', 'logs:' + name))
        manifest = next(download.rglob('manifest.json'))
        restored = restore_csv.restore(manifest)
        self.assertEqual(restored.read_bytes(), data)
        part = next(download.rglob('part-*'))
        part.write_bytes(b'corrupt')
        with self.assertRaisesRegex(ValueError, 'incomplete or corrupt'):
            restore_csv.restore(manifest, self.root / 'bad.csv')
        self.assertFalse((self.root / 'bad.csv').exists())

    def test_partial_upload_failure_retries_without_losing_confirmed_parts(self):
        data = os.urandom(5000)
        self.log(data=data)
        git = self.uploader.git
        pushes = 0
        def fail_later(*args, **kwargs):
            nonlocal pushes
            if args[0] == 'push':
                pushes += 1
                if pushes > 1:
                    return subprocess.CompletedProcess(args, 1, b'', b'network unavailable')
            return git(*args, **kwargs)
        with patch.object(upload, 'COMPRESS_AFTER_BYTES', 100), patch.object(upload, 'MAX_BLOB_BYTES', 2048), patch.object(upload, 'BATCH_BYTES', 2048):
            with patch.object(self.uploader, 'git', side_effect=fail_later), self.assertRaises(RuntimeError):
                self.uploader.sync()
            self.assertFalse((self.uploader.cache / 'uploaded.json').exists())
            self.assertFalse(any(name.endswith('manifest.json') for name in self.files()))
            self.assertEqual(self.uploader.sync(), 1)
            self.assertTrue(any(name.endswith('manifest.json') for name in self.files()))

    def test_batch_selection_leaves_remaining_csvs_for_next_timer_run(self):
        for index in range(3):
            self.log(f'SinglePendulum/run_logs/{index}.csv', b'12345678')
        with patch.object(upload, 'BATCH_BYTES', 10):
            self.assertEqual(self.uploader.sync(), 2)
            self.assertEqual(self.uploader.sync(), 1)
        self.assertEqual(len(self.files()), 3)

    def test_timeout_terminates_git_child_process_group(self):
        import sys
        marker = self.root / 'child-survived'
        child = 'import time; from pathlib import Path; time.sleep(1); Path(' + repr(str(marker)) + ').touch()'
        parent = 'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",' + repr(child) + ']); time.sleep(30)'
        with self.assertRaisesRegex(RuntimeError, 'exceeded'):
            upload.run_git([sys.executable, '-c', parent], env=dict(os.environ), timeout=0.15)
        time.sleep(1.1)
        self.assertFalse(marker.exists())


@unittest.skipUnless(Path('/proc').is_dir(), 'Linux process inspection')
class ActiveWriterTests(unittest.TestCase):
    def test_detects_writer_until_log_is_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp)
            path = repo / 'Code/SinglePendulum/run_logs/active.csv'
            path.parent.mkdir(parents=True)
            self.assertFalse(upload.logs_open_for_writing(repo))
            with path.open('w') as stream:
                stream.write('sample\n')
                self.assertTrue(upload.logs_open_for_writing(repo))
            self.assertFalse(upload.logs_open_for_writing(repo))


if __name__ == '__main__':
    unittest.main()
