#!/usr/bin/env python3
"""Stop the recorded localization owner and await its cleanup and lock release."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time


def lock_available(path):
    try:
        stream=Path(path).open('rb')
    except FileNotFoundError:
        return True
    with stream:
        try:
            fcntl.flock(stream,fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        return True


def process_identity(pid):
    try:
        args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        # Everything before the closing ')' is pid/comm, which may contain
        # spaces; field 22 is starttime, index 19 after that delimiter.
        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        if not any(args) or stat[0]=='Z':
            return None
        return args,int(stat[19])
    except (FileNotFoundError,ProcessLookupError):
        return None


def process_relationship(pid):
    """Read the kernel parent/group, never infer ownership from a PID alone."""
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return int(fields[1]),int(fields[2])
    except (FileNotFoundError,ProcessLookupError):
        return None


def holds_session_lock(pid, path):
    """Match the inode AND the inherited flock, not merely an open filename."""
    try:
        target=Path(path).stat()
        for fd in Path(f'/proc/{pid}/fd').iterdir():
            try:
                current=fd.stat()
                if (current.st_dev,current.st_ino)!=(target.st_dev,target.st_ino):
                    continue
                info=Path(f'/proc/{pid}/fdinfo/{fd.name}').read_text()
                if any(line.startswith('lock:') and 'FLOCK' in line and 'WRITE' in line
                       for line in info.splitlines()):
                    return True
            except OSError:
                continue
    except OSError:
        pass
    return False


def orphan_lock_holders(root, lock):
    """Allow recovery only after the recorded owner has exited.

    A process must actually hold this flock and belong to this checkout.
    Never infer permission from an obsolete PID or a generic ROS node name.
    """
    holders=[]
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name)==os.getpid():
            continue
        pid=int(process.name)
        try:
            if process.stat().st_uid!=os.getuid() or not holds_session_lock(pid,lock):
                continue
            identity=process_identity(pid)
            if identity is None:
                continue
            args,_=identity
            if any(Path(os.fsdecode(arg)).name=='run_r3_live_trial.sh' for arg in args if arg):
                raise RuntimeError(f'Phiên R3 PID {pid} đang giữ khóa nhưng chưa khớp tệp phiên; không dọn như tiến trình mồ côi.')
            # A project path in an arbitrary output argument is not ownership.
            # Check the executable/script or its actual working directory.
            prefix=os.fsencode(str(root)+'/')
            project=(any(arg.startswith(prefix) for arg in args[:2])
                     or (process/'cwd').resolve()==root)
            if not project:
                raise RuntimeError(f'Khóa R3 được PID {pid} ngoài dự án giữ; không tự gửi tín hiệu.')
            holders.append((pid,identity))
        except OSError:
            continue
    return holders


def interrupt_owner(pid,owner):
    """Unblock a foreground prompt while retaining the owner's cleanup traps.

    Bash defers a TERM trap while waiting for a foreground Python process.
    INT reaches Python's finally blocks as well as Bash's cleanup trap. Only
    a verified private group may receive a group signal; a shared terminal
    group is never interrupted.
    """
    if process_identity(pid)!=owner:
        return
    try:
        pgid=os.getpgid(pid)
    except ProcessLookupError:
        return
    if pgid==pid and pgid!=os.getpgrp():
        if process_identity(pid)==owner:
            try:
                os.killpg(pgid,signal.SIGINT)
            except ProcessLookupError:
                pass
        return

    # A shell may have launched the trial inside a larger foreground group.
    # Its direct children in that same group are foreground work; separately
    # sessioned ROS launch groups remain the owner's cleanup responsibility.
    children=[]
    try:
        entries=Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except (FileNotFoundError,ProcessLookupError):
        entries=[]
    for child in map(int,entries):
        if child==os.getpid() or process_relationship(child)!=(pid,pgid):
            continue
        identity=process_identity(child)
        if identity is not None:
            children.append((child,identity))
    if process_identity(pid)!=owner:
        return
    try:
        os.kill(pid,signal.SIGTERM)
    except ProcessLookupError:
        return
    for child,identity in children:
        if (process_identity(pid)==owner and process_identity(child)==identity
                and process_relationship(child)==(pid,pgid)):
            try:
                os.kill(child,signal.SIGINT)
            except ProcessLookupError:
                pass


def stop_session(root,timeout=90.,poll_interval=.05):
    if not math.isfinite(timeout) or timeout<=0 or not math.isfinite(poll_interval) or poll_interval<=0:
        raise ValueError('timeout and poll_interval must be finite and positive')
    root=Path(root).resolve()
    folder=root/'simulation/output/r3_live'
    session=folder/'localization_session.json'
    lock=folder/'.trial.lock'
    if lock_available(lock):
        return 'Khóa R3 đã được nhả; có thể chạy lượt mới.'
    owner=None;pid=None
    if session.is_file():
        try:
            state=json.loads(session.read_text())
            pid=state['pid']
            if isinstance(pid,bool) or not isinstance(pid,int) or pid<=1:
                raise ValueError('invalid process identity')
        except (KeyError,ValueError,TypeError) as exc:
            raise RuntimeError('Tệp phiên định vị không hợp lệ; không gửi tín hiệu.') from exc
        identity=process_identity(pid)
        if identity is not None:
            args,started=identity
            expected=str(root/'simulation/run_r3_live_trial.sh').encode()
            session_modes=(b'--localization-only',b'--nav-to-a',b'--nav-check-only',b'--recognize-here')
            if (expected not in args or not any(mode in args for mode in session_modes)
                    or ('process_start_ticks' in state and state['process_start_ticks']!=started)):
                raise RuntimeError('PID không còn thuộc phiên định vị; không gửi tín hiệu.')
            owner=identity
            interrupt_owner(pid,owner)
            print('Đang đóng phiên định vị, chờ các tiến trình con dừng và nhả khóa...',flush=True)
    deadline=time.monotonic()+timeout
    orphan_started=None
    next_orphan_scan=0.
    orphan_signals={}
    while True:
        same_owner=owner is not None and process_identity(pid)==owner
        if not same_owner and lock_available(lock):
            return 'Đã đóng phiên định vị và nhả khóa; có thể chạy lượt mới.'
        now=time.monotonic()
        if not same_owner and now>=next_orphan_scan:
            holders=orphan_lock_holders(root,lock)
            next_orphan_scan=now+.25
            if not holders:
                if lock_available(lock):
                    return 'Đã nhả khóa R3; có thể chạy lượt mới.'
                raise RuntimeError('Chủ phiên đã thoát nhưng khóa còn bị giữ; không xác minh được tiến trình giữ khóa.')
            if orphan_started is None:
                orphan_started=now
                print('Chủ phiên đã thoát; đóng tiến trình con còn giữ khóa: '
                      +', '.join(str(p) for p,_ in holders),flush=True)
            elapsed=now-orphan_started
            sig=signal.SIGKILL if elapsed>=4. else signal.SIGTERM if elapsed>=2. else signal.SIGINT
            for orphan,identity in holders:
                key=(orphan,identity[1])
                if (orphan_signals.get(key)!=sig and process_identity(orphan)==identity
                        and holds_session_lock(orphan,lock)):
                    try:
                        os.kill(orphan,sig)
                        orphan_signals[key]=sig
                    except ProcessLookupError:
                        pass
        if time.monotonic()>=deadline:
            raise RuntimeError(f'Chưa đóng hoàn toàn sau {timeout:g} giây; khóa R3 vẫn có thể đang được giữ. '
                               'Chưa được chạy lượt mới; kiểm tra tiến trình của phiên cũ.')
        time.sleep(min(poll_interval,max(0.,deadline-time.monotonic())))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--timeout',type=float,default=90.)
    args=parser.parse_args()
    try:
        print(stop_session(args.root,args.timeout),flush=True)
        return 0
    except (RuntimeError,ValueError,OSError) as exc:
        print(str(exc),flush=True)
        return 2
    except KeyboardInterrupt:
        print('Đã ngắt lệnh chờ dừng; phiên R3 có thể chưa đóng hết.',flush=True)
        return 130


if __name__=='__main__':
    raise SystemExit(main())
