#!/usr/bin/env python3
"""Check lifecycle states and pause or resume the existing Nav2 nodes."""
import argparse
import math
from pathlib import Path
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


NAMES=('map_server','amcl')
REQUEST_TIMEOUT_S=2.
STARTUP_FAILURE_MARKERS=('Failed to bring up all requested nodes',
                         'Failed to change state for node:')


def readiness(states, now, max_age_s=2.5, *, names=NAMES, expected_state=3):
    """Require a recent response in the expected state from every node."""
    label='ACTIVE' if expected_state==3 else 'INACTIVE'
    for name in names:
        state=states.get(name,{})
        if state.get('error'):
            return False,f'{name}: {state["error"]}'
        stamp=state.get('stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not 0<=now-stamp<=max_age_s:
            return False,f'{name}: missing/stale lifecycle response'
        if state.get('id')!=expected_state:
            return False,f'{name}: lifecycle state {state.get("id")} is not {label} ({expected_state})'
    return True,' and '.join(names)+' lifecycle '+label


class LifecyclePoll:
    def __init__(self, clients, *, poll_period_s=.25, recreate_client=None):
        self.clients=clients
        self.poll_period_s=poll_period_s
        self.recreate_client=recreate_client
        self.states={}
        self.pending={}
        self.next_poll={name:0. for name in clients}
        self.failures={name:0 for name in clients}
        self.timeouts={name:0 for name in clients}
        self.recreated={name:0 for name in clients}

    def failed(self,name,wall,reason):
        self.states[name]=dict(error=reason,stamp=wall)
        self.failures[name]+=1
        self.next_poll[name]=wall+self.poll_period_s
        if self.recreate_client is not None and self.failures[name]>=2:
            try:
                self.clients[name]=self.recreate_client(name,self.clients[name])
            except Exception as exc:
                self.states[name]['error']+='; client recreation failed: '+str(exc)
            else:
                self.recreated[name]+=1
                self.failures[name]=0

    def diagnostics(self,wall):
        return {name:dict(self.states.get(name,{}),
                         request_pending=name in self.pending,
                         request_age_s=(max(0.,wall-self.pending[name][1])
                                        if name in self.pending else None),
                         request_timeouts=self.timeouts[name],client_recreations=self.recreated[name])
                for name in self.clients}

    def tick(self, wall):
        for name in self.clients:
            client=self.clients[name]
            try:
                if name in self.pending:
                    future,sent=self.pending[name]
                    if future.done():
                        del self.pending[name]
                        response=future.result()
                        self.states[name]=dict(id=response.current_state.id,stamp=wall)
                        self.failures[name]=0
                        self.next_poll[name]=wall+self.poll_period_s
                    elif wall-sent>=REQUEST_TIMEOUT_S:
                        del self.pending[name]
                        client.remove_pending_request(future)
                        if hasattr(future,'cancel'):
                            future.cancel()
                        self.timeouts[name]+=1
                        self.failed(name,wall,'get_state request timed out; retrying')
                    continue
                if wall<self.next_poll[name]:
                    continue
                self.next_poll[name]=wall+self.poll_period_s
                if not client.service_is_ready():
                    self.states[name]=dict(error='get_state service unavailable')
                    continue
                self.pending[name]=(client.call_async(GetState.Request()),wall)
            except Exception as exc:
                self.pending.pop(name,None)
                self.failed(name,wall,'get_state failed: '+str(exc))


def startup_failure(log_path):
    if log_path is None:
        return None
    try:
        lines=Path(log_path).read_text(errors='replace').splitlines()
    except OSError:
        return None
    return next((line for line in reversed(lines)
                 if any(marker in line for marker in STARTUP_FAILURE_MARKERS)),None)


def wait_active(poll, timeout, spin, *, clock=time.monotonic, ok=rclpy.ok, report=print,
                startup_log=None, expected_state=3):
    deadline=clock()+timeout
    last_reason=None
    waiting='Waiting for Nav2 to pause: ' if expected_state==2 else 'Waiting for localization activation: '
    while ok() and clock()<deadline:
        failure=startup_failure(startup_log)
        if failure is not None:
            report('Lifecycle startup aborted; owned group must be restarted: '+failure)
            return False
        wall=clock()
        poll.tick(wall)
        ready,reason=readiness(poll.states,wall,names=tuple(poll.clients),expected_state=expected_state)
        if ready:
            report(reason)
            return True
        if reason!=last_reason:
            report(waiting+reason)
            last_reason=reason
        spin(min(.05,max(0.,deadline-clock())))
    report(waiting+'timed out or stopped: '+str(last_reason))
    return False


def navigation_transition(node, executor, transition, deadline):
    client=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
    remaining=max(0.,deadline-time.monotonic())
    if not client.wait_for_service(timeout_sec=remaining):
        raise RuntimeError('Không tìm thấy dịch vụ lifecycle của Nav2.')
    request=ManageLifecycleNodes.Request()
    request.command=request.PAUSE if transition=='pause' else request.RESUME
    future=client.call_async(request)
    executor.spin_until_future_complete(future,timeout_sec=max(0.,deadline-time.monotonic()))
    if not future.done() or future.result() is None or not future.result().success:
        raise RuntimeError('Nav2 không xác nhận thao tác '+transition)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=float,default=30.)
    parser.add_argument('--nodes',nargs='+',default=list(NAMES),
                        help='Lifecycle node names whose ACTIVE state is required')
    parser.add_argument('--startup-log',type=Path,
                        help='Exit early if this attempt\'s lifecycle manager aborts bringup')
    parser.add_argument('--navigation-transition',choices=('pause','resume'))
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0:
        parser.error('--timeout must be finite and positive')
    names=tuple(name.strip('/') for name in args.nodes)
    if not names or any(not name or '//' in name for name in names) or len(set(names))!=len(names):
        parser.error('--nodes requires distinct nonempty node names')
    rclpy.init()
    node=Node('r3_wait_localization_active')
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        deadline=time.monotonic()+args.timeout
        if args.navigation_transition:
            navigation_transition(node,executor,args.navigation_transition,deadline)
        def recreate(name,client):
            node.destroy_client(client)
            return node.create_client(GetState,f'/{name}/get_state')
        poll=LifecyclePoll({name:node.create_client(GetState,f'/{name}/get_state') for name in names},
                           poll_period_s=.5,recreate_client=recreate)
        active=wait_active(poll,max(0.,deadline-time.monotonic()),
                           lambda seconds:executor.spin_once(timeout_sec=seconds),
                           report=lambda message:print(message,flush=True),startup_log=args.startup_log,
                           expected_state=2 if args.navigation_transition=='pause' else 3)
        return 0 if active else 2
    except RuntimeError as exc:
        print(str(exc),flush=True)
        return 2
    except KeyboardInterrupt:
        return 2
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
