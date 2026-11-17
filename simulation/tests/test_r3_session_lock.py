"""Exercise real flock/fd inheritance with stub processes, never ROS/hardware."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from r3_stop_localization import (holds_session_lock, lock_available, process_identity,
                                  stop_session)

LAUNCHER = Path(__file__).resolve().parents[1]/'run_r3_live_trial.sh'


class SessionLockTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='r3-lock-test-')
        self.root = Path(self.temp.name)/'project'
        self.folder = self.root/'simulation/output/r3_live'
        self.folder.mkdir(parents=True)
        self.lock = self.folder/'.trial.lock'
        self.processes = []
        self.camera_pids = []

    def tearDown(self):
        for pid in self.camera_pids:
            identity = process_identity(pid)
            if identity is not None and any(b'fake_camera.py' in arg for arg in identity[0]):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for child in self.processes:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=2.)
        self.temp.cleanup()

    def spawn(self, *, original=False, foreground=False, outside=False, take_lock=True):
        child_dir = Path(self.temp.name)/'outside' if outside else self.root
        child_dir.mkdir(exist_ok=True)
        fake = child_dir/'fake_camera.py'
        marker = self.root/'child.json'
        fake.write_text('''import json,os,sys,time
from pathlib import Path
path=Path(sys.argv[1])
try:
    os.fstat(9); inherited=True
except OSError:
    inherited=False
path.write_text(json.dumps(dict(pid=os.getpid(),fd9=inherited)))
while True:
    time.sleep(.05)
''')
        source = LAUNCHER.read_text()
        if foreground:
            command = re.search(r'(?m)^python3 9>&- simulation/r3_real_trial\.py[^\n]*\n[^\n]*', source).group()
            stub = 'python3() { exec /usr/bin/python3 "$fake" "$marker"; }'
        else:
            command = re.search(r'(?m)^  ASTRA_POINT_CLOUD=false setsid[^\n]*\n[^\n]*\n  camera_pid=\$!', source).group()
            # Override setsid itself: the real camera launch is never executed.
            stub = 'setsid() { exec /usr/bin/python3 "$fake" "$marker"; }'
        if original:
            command = command.replace(' 9>&-', '')
        script = self.root/'owner.sh'
        prefix = '''set -eu
lock=$1
fake=$2
marker=$3
run_dir=$4
plan=unused_stub_plan
trial_mode=(--pickup)
exec 9>"$lock"
'''
        if take_lock:
            prefix += 'flock -n 9\n'
        body = prefix+stub+'\n'+command+'\n'
        if not foreground:
            body += 'for i in {1..300}; do [[ ! -s "$marker" ]] || exit 0; sleep .01; done\nexit 2\n'
        script.write_text(body)
        log = (self.root/'owner.log').open('wb')
        try:
            owner = subprocess.Popen(['bash', str(script), str(self.lock), str(fake),
                                      str(marker), str(self.root)], cwd=child_dir,
                                     start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
        finally:
            log.close()
        self.processes.append(owner)
        deadline = time.monotonic()+4.
        while not marker.is_file() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(marker.is_file(), (self.root/'owner.log').read_text())
        child = json.loads(marker.read_text())
        self.camera_pids.append(child['pid'])
        if foreground:
            os.kill(owner.pid, signal.SIGKILL)
        self.assertIn(owner.wait(timeout=2.), (0, -signal.SIGKILL))
        (self.folder/'localization_session.json').write_text(json.dumps(dict(pid=owner.pid)))
        return child

    def test_original_camera_inherits_lock_after_owner_exit(self):
        child = self.spawn(original=True)
        self.assertTrue(child['fd9'])
        self.assertTrue(holds_session_lock(child['pid'], self.lock))
        self.assertFalse(lock_available(self.lock))

    def test_actual_camera_launch_redirection_releases_lock_with_owner(self):
        child = self.spawn()
        self.assertFalse(child['fd9'])
        self.assertFalse(holds_session_lock(child['pid'], self.lock))
        self.assertTrue(lock_available(self.lock))

    def test_actual_foreground_trial_and_tee_do_not_retain_lock(self):
        child = self.spawn(foreground=True)
        self.assertFalse(child['fd9'])
        self.assertTrue(lock_available(self.lock))

    def test_stop_recovers_verified_orphan_holding_old_lock(self):
        child = self.spawn(original=True)
        message = stop_session(self.root, timeout=3., poll_interval=.02)
        self.assertIn('có thể chạy lượt mới', message)
        self.assertTrue(lock_available(self.lock))
        self.assertFalse(holds_session_lock(child['pid'], self.lock))

    def test_stop_refuses_holder_outside_project(self):
        child = self.spawn(original=True, outside=True)
        with self.assertRaisesRegex(RuntimeError, 'ngoài dự án'):
            stop_session(self.root, timeout=1.)
        self.assertIsNotNone(process_identity(child['pid']))
        self.assertFalse(lock_available(self.lock))

    def test_open_fd_without_flock_is_not_a_lock_holder(self):
        child = self.spawn(original=True, take_lock=False)
        self.assertTrue(child['fd9'])
        self.assertFalse(holds_session_lock(child['pid'], self.lock))
        self.assertTrue(lock_available(self.lock))

    def test_unlocked_stale_session_is_idempotent(self):
        (self.folder/'localization_session.json').write_text(json.dumps(dict(pid=2147483647)))
        self.assertIn('có thể chạy lượt mới', stop_session(self.root, timeout=.1))
        self.assertIn('có thể chạy lượt mới', stop_session(self.root, timeout=.1))


if __name__ == '__main__':
    unittest.main()
