"""Mock-only safety tests: no ROS graph, serial ports, or robot motion."""
import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
import math
import pytest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

loader = SourceFileLoader('navigation_safety_impl', str(Path(__file__).parents[1]/'scripts/navigation_safety'))
spec = importlib.util.spec_from_loader(loader.name, loader)
impl = importlib.util.module_from_spec(spec)
loader.exec_module(impl)
Safety = impl.NavigationSafety


def subject():
    now = time.monotonic()
    obj = NS(mode=0, reason='', pending_free=None, pending_drive=None, stopped_since=None,
             ready_since=now-3, armed_at=math.inf, drive_source='navigation',
             raw={'navigation': Twist(), 'manual': Twist()},
             last_command={'navigation': -math.inf, 'manual': -math.inf},
             a=0, b=1, x=2, y=3, buttons=[0]*11,
             output=Mock(), lease=Mock(), status=Mock(),
             get_logger=Mock(), problem=lambda: None, physical_problem=lambda: None,
             active={}, cancel_futures={}, rpm=[0, 0], last_tick=now,
             cancel_goals=Mock(), header_ok=lambda *args: True,
             ids=[1, 2], fresh={k: now for k in ('joy', 'rpm', 'online')})
    obj.stop = lambda reason: Safety.stop(obj, reason)
    obj.request_free = lambda: Safety.request_free(obj)
    obj.activate_drive = lambda source: Safety.activate_drive(obj, source)
    obj.request_drive = lambda source: Safety.request_drive(obj, source)
    return obj


def test_startup_zero_and_arm():
    obj = subject()
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0
    reply = Safety.arm(obj, None, NS(success=False, message=''))
    assert reply.success and obj.mode == 2


def test_b_beats_other_buttons():
    obj = subject(); obj.mode = 2
    msg = Joy(); msg.buttons = [1]*11
    Safety.joy(obj, msg)
    assert obj.mode == 0 and obj.pending_free is None
    assert obj.lease.publish.call_args.args[0].data == 0
    msg.buttons = [1, 0, 1, 0]+[0]*7
    Safety.joy(obj, msg)
    assert obj.mode == 0


def test_y_waits_for_wheels_to_stop():
    obj = subject(); obj.mode = 2; obj.rpm = [50, 50]
    msg = Joy(); msg.buttons = [0, 0, 0, 1]+[0]*7
    Safety.joy(obj, msg)
    Safety.tick(obj)
    assert obj.mode == 0 and obj.pending_free is not None
    obj.rpm = [0, 0]; obj.stopped_since = time.monotonic()-.4
    Safety.tick(obj)
    assert obj.mode == 1


def test_free_times_out_when_spinning():
    obj = subject(); obj.pending_free = time.monotonic()-4; obj.rpm = [30, 30]
    Safety.tick(obj)
    assert obj.mode == 0 and obj.pending_free is None


def test_odom_or_tf_fault_latches_and_does_not_resume():
    obj = subject(); obj.mode = 2
    obj.problem = lambda: 'odom stale'
    Safety.tick(obj)
    assert obj.mode == 0
    obj.problem = lambda: None
    Safety.tick(obj)
    assert obj.mode == 0


def test_free_brakes_on_joy_loss():
    obj = subject(); obj.mode = 1
    obj.physical_problem = lambda: 'joy stale'
    Safety.tick(obj)
    assert obj.mode == 0


def test_existing_goal_can_be_explicitly_rearmed():
    obj = subject(); obj.active = {'navigate_to_pose': True}
    reply = Safety.arm(obj, None, NS(success=False, message=''))
    assert reply.success and obj.drive_source == 'navigation'


def test_a_selects_navigation_and_x_selects_manual():
    obj = subject()
    msg = Joy(); msg.buttons = [1, 0, 0, 0]+[0]*7
    Safety.joy(obj, msg)
    assert obj.pending_drive == 'navigation'
    msg.buttons = [0]*11; Safety.joy(obj, msg)
    obj.ready_since = time.monotonic()-3
    msg.buttons = [0, 0, 1, 0]+[0]*7
    Safety.joy(obj, msg)
    Safety.tick(obj)
    assert obj.mode == 2 and obj.drive_source == 'manual'


def test_old_commands_never_replayed():
    obj = subject()
    obj.raw['navigation'].linear.x = .2
    obj.last_command['navigation'] = time.monotonic()-1
    assert Safety.arm(obj, None, NS(success=False, message='')).success
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0.
    msg = Twist(); msg.linear.x = .1
    Safety.command(obj, 'navigation', msg); Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == .1
    obj.last_command['navigation'] -= 1
    Safety.tick(obj)
    assert obj.output.publish.call_args.args[0].linear.x == 0.


def test_timer_stall_brakes():
    obj = subject(); obj.mode = 2; obj.last_tick -= 1
    Safety.tick(obj)
    assert obj.mode == 0


def test_invalid_commands_brake():
    obj = subject(); obj.mode = 2
    msg = Twist(); msg.linear.x = float('nan')
    Safety.command(obj, 'navigation', msg)
    assert obj.mode == 0


def test_physical_freshness():
    obj = subject()
    assert Safety.physical_problem(obj) is None
    obj.fresh['joy'] -= 1
    assert 'joy' in Safety.physical_problem(obj)


@pytest.mark.parametrize('age', [29., -3., 100.])
def test_stale_or_future_map_tf(age):
    obj = subject()
    obj.fresh.update(odom=time.monotonic(), scan=time.monotonic())
    obj.get_name = lambda: 'navigation_safety'
    obj.get_publishers_info_by_topic = lambda topic: [NS(node_name=(
        'two_wheels_robot_node' if topic.endswith('rpm_cmd') else
        'collision_monitor' if topic.endswith('cmd_vel_collision') else
        'velocity_smoother_manual' if topic.endswith('cmd_vel_teleop') else
        'navigation_safety'))]
    obj.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
    obj.buffer = Mock()
    def lookup(parent, child, stamp):
        seconds = 100.-age if parent == 'map' else 100.
        return NS(header=NS(stamp=NS(sec=int(seconds), nanosec=0)))
    obj.buffer.lookup_transform.side_effect = lookup
    assert 'map -> odom TF' in Safety.problem(obj)


def test_free_to_drive_request_brakes_then_arms_when_stationary():
    obj = subject(); obj.mode = 1
    Safety.request_drive(obj, 'navigation')
    assert obj.mode == 0 and obj.pending_drive == 'navigation'
    obj.ready_since = time.monotonic()-3
    Safety.tick(obj)
    assert obj.mode == 2 and obj.drive_source == 'navigation'
