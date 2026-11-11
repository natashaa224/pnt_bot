#!/usr/bin/env python3
"""
actuator_bridge_node.py

Bridges paint_executor_node and wall_motion_node commands to hardware or simulation.
Backend: sim | sbc — selected by 'hardware_backend' param.

Subscribes:
    /z_axis_cmd                      (painting_robot_msgs/ZAxisCmd)
    /wrist_cmd                       (painting_robot_msgs/WristCmd)
    /spray_cmd                       (painting_robot_msgs/SprayCmd)
    /paintbot_real/cmd_vel (geometry_msgs/Twist)  [sbc only]

Publishes:
    /paint_head_status  (painting_robot_msgs/PaintHeadStatus)
    /odom               (nav_msgs/Odometry)                 [sbc only]
"""

import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from painting_robot_msgs.msg import ZAxisCmd, WristCmd, SprayCmd, PaintHeadStatus, GantryLimitCmd, SafetyCommand
import tf2_ros
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Backend base
# ---------------------------------------------------------------------------
class ActuatorBackend:
    def __init__(self):
        # Shared homing contract — every backend answers these
        self._homed  = False
        self._homing = False

    def move_z(self, target_m: float, rpm: int):                    raise NotImplementedError
    def stop_z(self):                                               raise NotImplementedError
    def home_z(self):                                               pass
    def get_z_state(self, read_hw: bool = False) -> dict:           raise NotImplementedError
    def set_spray(self, on: bool):                                  raise NotImplementedError
    def set_wrist(self, angle_deg: float):                          raise NotImplementedError
    def get_wrist_deg(self) -> float:                               return 0.0
    def drive_base(self, linear_mps: float, angular_radps: float):  pass
    def stop_base(self):                                            pass
    def read_base_encoders(self) -> tuple:                          return (0, 0)
    def spin_once(self):                                            pass
    def clear_limits(self):                                         pass

# ---------------------------------------------------------------------------
# Sim backend
# ---------------------------------------------------------------------------
class SimBackend(ActuatorBackend):

    def __init__(self, node: Node):
        super().__init__()
        self._node            = node
        self._z_max_m         = node.get_parameter('z_max_m').value
        self._sim_speed_mps   = node.get_parameter('z_sim_speed_mps').value
        self._z_target        = 0.0
        self._moving          = False
        self._js_z_pos        = 0.0
        self._sim_z_pos       = 0.0
        self._move_start_time = None
        self._move_duration   = 1.0
        self._jog_dir       = 0
        self._jog_speed_mps = float(node.get_parameter('z_sim_speed_mps').value)
        self._jog_target    = 0.0
        self._jog_last_t    = None

        # Manual teach-homing state — pre-homed in simulation so painting stack starts ready
        self._teaching        = False
        self._lower_set       = True
        self._upper_set       = True
        self._homed           = True

        self._traj_pub = node.create_publisher(
            JointTrajectory,
            '/paint_axis_controller/joint_trajectory', 10)

        node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _joint_state_cb(self, msg: JointState):
        if 'zAxis_joint' in msg.name:
            idx = msg.name.index('zAxis_joint')
            self._js_z_pos = msg.position[idx]

    def move_z(self, target_m: float, rpm: int):
        target_m              = max(0.0, min(self._z_max_m, target_m))
        self._z_target        = target_m
        self._moving          = True
        self._move_last_t     = self._now_s()
        self._sim_z_pos       = self._js_z_pos
        dist                  = abs(target_m - self._js_z_pos)
        self._move_duration   = max(0.5, dist / self._sim_speed_mps)

        self._node.get_logger().info(
            f'[sim] Z move → {target_m:.3f}m from {self._js_z_pos:.3f}m (est {self._move_duration:.1f}s)')

    def stop_z(self):
        self._moving    = False
        self._z_target  = self._js_z_pos
        self._sim_z_pos = self._js_z_pos
        self._jog_dir   = 0
        self._publish_z_point(self._js_z_pos)
        self._node.get_logger().info('[sim] Z stop')


    def clear_limits(self):
        self._homed     = False
        self._lower_set = False
        self._node.get_logger().info('[sim] gantry limits cleared (re-teach required)')    

    def home_z(self):
        """
        Arm manual teach-homing. No physical limit switches exist on this
        machine — the operator jogs the gantry (D-pad) and presses
        set_lower (A) then set_upper (Y) to teach the travel range.
        """
        self._teaching  = True
        self._homed     = False
        self._lower_set = False
        self._upper_set = False
        self._node.get_logger().info(
            f'[sim] teach-homing armed, js_z_pos={self._js_z_pos:.4f} — '
            f'jog down and press A, then jog up and press Y')

    def teach_set_lower(self):
        """Record current joint position as the soft lower limit. Sim's
        joint origin (0.0) is the fixed mechanical home, so no separate
        zero datum needs to be captured here (unlike the real backend)."""
        self._lower_set = True
        return self._js_z_pos * 1000.0

    def teach_set_upper(self):
        if not self._homed and not self._lower_set:
            self._node.get_logger().warn('[sim] set UPPER rejected — set LOWER (A) first')
            return None
        pos_mm = self._js_z_pos * 1000.0
        if not self._homed:
            self._upper_set = True
            self._teaching  = False
            self._homed     = True
        return pos_mm

    def get_z_state(self, read_hw: bool = False) -> dict:
        return {
            'position_m':   self._js_z_pos,
            'velocity_mps': 0.0,
            'moving':       self._moving,
            'fault':        False,
            'homed':        self._homed,
        }

    def set_spray(self, on: bool):
        self._node.get_logger().info(f'[sim] spray → {"ON" if on else "OFF"}')

    def set_wrist(self, angle_deg: float):
        self._wrist_deg = angle_deg
        self._node.get_logger().info(f'[sim] wrist → {angle_deg:.1f}°',
                                     throttle_duration_sec=0.5)
        self._publish_z_point(getattr(self, '_sim_z_pos', self._js_z_pos))

    def get_wrist_deg(self) -> float:
        return getattr(self, '_wrist_deg', 0.0)

    def drive_base(self, linear_mps: float, angular_radps: float):
        self._node.get_logger().debug(
            f'[sim] base vel  lin={linear_mps:.3f}  ang={angular_radps:.3f}')

    def stop_base(self):
        self._node.get_logger().debug('[sim] base stop')

    def jog_z(self, velocity_rpm: int):
        self._jog_dir = -1 if velocity_rpm < 0 else (1 if velocity_rpm > 0 else 0)
        if self._jog_dir == 0:
            self._moving = False
            return
        self._sim_z_pos  = self._js_z_pos
        self._jog_target = self._js_z_pos
        self._jog_last_t = self._now_s()
        self._moving     = True
        self._node.get_logger().info(f'[sim] jog_z dir={self._jog_dir:+d}')

    def _publish_z_point(self, target_m: float):
        msg = JointTrajectory()
        msg.header.frame_id = 'base_footprint'
        msg.joint_names = ['zAxis_joint', 'wrist_joint']
        pt  = JointTrajectoryPoint()
        wrist_rad = math.radians(getattr(self, '_wrist_deg', 0.0))
        pt.positions       = [float(target_m), float(wrist_rad)]
        pt.time_from_start = Duration(sec=0, nanosec=0)
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def spin_once(self):
        # No autonomous homing motion — homing is taught by the operator
        # via home_z()/teach_set_lower()/teach_set_upper(), not polled here.
        now = self._now_s()
        if self._jog_dir != 0:
            dt  = (now - self._jog_last_t) if self._jog_last_t else 0.0
            self._jog_last_t = now
            if dt > 0.0:
                self._jog_target += self._jog_dir * self._jog_speed_mps * dt
                self._jog_target  = max(0.0, min(self._z_max_m, self._jog_target))
                self._sim_z_pos   = self._jog_target
                self._publish_z_point(self._sim_z_pos)

        elif self._moving:
            dt = (now - getattr(self, '_move_last_t', now))
            self._move_last_t = now
            err = self._z_target - getattr(self, '_sim_z_pos', self._js_z_pos)
            if abs(err) <= 0.005:
                self._sim_z_pos = self._z_target
                self._moving = False
                self._publish_z_point(self._sim_z_pos)
            elif dt > 0.0:
                step = math.copysign(min(abs(err), self._sim_speed_mps * dt), err)
                self._sim_z_pos = getattr(self, '_sim_z_pos', self._js_z_pos) + step
                self._sim_z_pos = max(0.0, min(self._z_max_m, self._sim_z_pos))
                self._publish_z_point(self._sim_z_pos)


# ---------------------------------------------------------------------------
# SBC backend
# ---------------------------------------------------------------------------
class SBCBackend(ActuatorBackend):

    def __init__(self, node: Node):
        super().__init__()
        self._node       = node
        self._z          = None
        self._base       = None
        self._gpio       = None
        self._gpio_ready = False
        self._pin        = node.get_parameter('solenoid_gpio_pin').value

        self._last_drive_key    = None
        self._last_drive_send_t = 0.0
        self._keepalive_moving_s = float(node.get_parameter('keepalive_moving_s').value)
        self._keepalive_idle_s   = float(node.get_parameter('keepalive_idle_s').value)

        # Z state cache — updated only when motion is active
        self._z_pos_cache_mm = 0.0

        # Homing / zero datum — all Z positions are relative to this offset.
        # No physical limit switches exist on this machine — homing is
        # taught by the operator (see home_z/teach_set_lower/teach_set_upper).
        self._home_offset_counts = 0
        self._teaching           = False
        self._lower_set          = False
        self._upper_set          = False

        # --- Serial port (shared by ZAxis + BaseDrive) ---
        # timeout=0.05 — short so RS485 reads never block the executor
        ser = None
        try:
            import serial
            ser = serial.Serial(
                port=node.get_parameter('serial_port').value,
                baudrate=node.get_parameter('serial_baud').value,
                timeout=0.05)
            node.get_logger().info(
                f'[sbc] RS485 opened: {node.get_parameter("serial_port").value}')
        except Exception as e:
            node.get_logger().error(f'[sbc] RS485 open failed: {e}')

        # --- ZAxis ---
        if ser:
            try:
                from painting_robot_painting.leadshine_driver import ZAxis
                # self._z = ZAxis(
                #     ser=ser,
                #     slave_id=int(node.get_parameter('z_slave_id').value),
                #     counts_per_mm=float(node.get_parameter('z_counts_per_mm').value),
                #     min_height_mm=0.0,
                #     max_height_mm=float(node.get_parameter('strip_z_max').value) * 1000.0)
                self._z = ZAxis(
                    ser=ser,
                    slave_id=int(node.get_parameter('z_slave_id').value),
                    counts_per_mm=float(node.get_parameter('z_counts_per_mm').value),
                    min_height_mm=0.0,
                    max_height_mm=float(node.get_parameter('strip_z_max').value) * 1000.0,
                    accel=int(node.get_parameter('z_accel_up_ms').value),
                    decel=int(node.get_parameter('z_decel_up_ms').value))
                try:
                    pos = self._z.read_position_mm()
                    node.get_logger().info(f'[sbc] Leadshine ZAxis initialised — probe pos={pos:.1f}mm')
                except Exception as e:
                    node.get_logger().warn(f'[sbc] ZAxis probe failed — disabling: {e}')
                    self._z = None
            except Exception as e:
                node.get_logger().error(f'[sbc] ZAxis init failed: {e}')

                

        # --- BaseDrive ---
        if ser:
            try:
                from painting_robot_painting.leadshine_driver import BaseDrive
                self._base = BaseDrive(
                    ser=ser,
                    left_id=int(node.get_parameter('base_left_id').value),
                    right_id=int(node.get_parameter('base_right_id').value),
                    wheel_dia_m=float(node.get_parameter('wheel_dia_m').value),
                    track_width_m=float(node.get_parameter('track_width_m').value),
                    gear_ratio=float(node.get_parameter('base_gear_ratio').value),
                    counts_per_rev=int(node.get_parameter('base_counts_per_rev').value),
                    left_sign=1,
                    right_sign=-1,
                )
                try:
                    l, r = self._base.read_encoders()
                    node.get_logger().info(f'[sbc] BaseDrive initialised — probe L:{l} R:{r}')
                    self._enc_offset_l = 0
                    self._enc_offset_r = 0
                    if node.get_parameter('zero_encoders_on_start').value:
                        self._enc_offset_l = l
                        self._enc_offset_r = r
                        node.get_logger().info(f'[sbc] encoders zeroed — L:{l}  R:{r}')
                except Exception as e:
                    node.get_logger().warn(f'[sbc] BaseDrive probe failed — disabling: {e}')
                    self._base = None
            except Exception as e:
                node.get_logger().error(f'[sbc] BaseDrive init failed: {e}')

        # --- GPIO solenoid ---
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._pin, GPIO.OUT, initial=GPIO.LOW)
            self._gpio       = GPIO
            self._gpio_ready = True
            node.get_logger().info(f'[sbc] GPIO solenoid pin {self._pin}')
        except Exception as e:
            node.get_logger().warn(f'[sbc] GPIO not available: {e}')

                # --- Wrist servo ---
        self._wrist = None

        try:
            import serial as _serial
            wrist_ser = _serial.Serial(
                port=node.get_parameter('wrist_port').value,
                baudrate=1_000_000,
                timeout=0.1)
            from painting_robot_painting.wrist_driver import WristServo
            self._wrist = WristServo(
                ser=wrist_ser,
                servo_id=int(node.get_parameter('wrist_servo_id').value),
                center_pos=int(node.get_parameter('wrist_center_pos').value),
                counts_per_deg=float(node.get_parameter('wrist_counts_per_deg').value),
                min_pos=int(node.get_parameter('wrist_min_pos').value),
                max_pos=int(node.get_parameter('wrist_max_pos').value),
            )
            self._wrist.torque_enable(True)
            node.get_logger().info(
                f'[sbc] WristServo initialised on {node.get_parameter("wrist_port").value}')
        except Exception as e:
            node.get_logger().warn(f'[sbc] Wrist init failed: {e}')    

    # --- Z-axis ---
    def move_z(self, target_m: float, rpm: int):
        target_mm = target_m * 1000.0
        if self._z:
            counts = int(target_mm * self._z.counts_per_mm) + self._home_offset_counts
            self._node.get_logger().info(
                f'[sbc] move_z → {target_mm:.1f}mm (counts={counts}, homed={self._homed})')
            self._z.motor.command_position(counts, rpm, relative=False)
        else:
            self._node.get_logger().warn('[sbc] move_z — no ZAxis instance')

    def jog_z(self, velocity_rpm: int):
        if self._z:
            self._z.motor.command_velocity(int(velocity_rpm))
        else:
            self._node.get_logger().warn('[sbc] jog_z — no ZAxis instance')

    def clear_limits(self):
        """Clear taught gantry limits — operator must re-teach after a fault."""
        self._homed              = False
        self._lower_set          = False
        self._home_offset_counts = 0
        self._node.get_logger().info('[sbc] gantry limits cleared (re-teach required)')        

    def stop_z(self):
        self._node.get_logger().info('[sbc] stop_z called')
        if self._z:
            self._z.stop()
        else:
            self._node.get_logger().warn('[sbc] stop_z — no ZAxis instance')

    def home_z(self):
        """
        Arm manual teach-homing. No physical limit switches exist on this
        machine — the operator jogs the gantry (D-pad) and presses
        set_lower (A) then set_upper (Y) to teach the travel range.
        """
        if not self._z:
            self._node.get_logger().warn('[sbc] home_z — no ZAxis instance')
            return
        self.stop_z()
        self._teaching  = True
        self._homed     = False
        self._lower_set = False
        self._upper_set = False
        self._node.get_logger().info(
            '[sbc] teach-homing armed — jog down and press A (set lower), '
            'then jog up and press Y (set upper)')

    def teach_set_lower(self):
        """
        First call (pre-homing): captures the current encoder count as the
        zero datum — this IS the homing reference, replacing the physical
        lower limit switch this machine doesn't have.
        After homing: just re-teaches the soft lower bound relative to the
        already-established zero, without moving the zero datum.
        Returns the resulting soft-lower value in mm, or None on failure.
        """
        if not self._z:
            self._node.get_logger().warn('[sbc] teach_set_lower — no ZAxis instance')
            return None
        self.stop_z()
        if not self._homed:
            self._home_offset_counts = self._z.motor.read_encoder()
            self._z_pos_cache_mm     = 0.0
            self._lower_set          = True
            self._node.get_logger().info(
                f'[sbc] LOWER limit set — offset={self._home_offset_counts} counts set as zero')
            return 0.0
        pos_mm = self.get_z_state(read_hw=True)['position_m'] * 1000.0
        return pos_mm

    def teach_set_upper(self):
        """
        First call (pre-homing, requires lower already set): captures the
        current position as the soft upper bound and completes homing.
        After homing: just re-teaches the soft upper bound.
        Returns the resulting soft-upper value in mm, or None on failure.
        """
        if not self._z:
            self._node.get_logger().warn('[sbc] teach_set_upper — no ZAxis instance')
            return None
        if not self._homed and not self._lower_set:
            self._node.get_logger().warn('[sbc] set UPPER rejected — set LOWER (A) first')
            return None
        self.stop_z()
        pos_mm = self.get_z_state(read_hw=True)['position_m'] * 1000.0
        if not self._homed:
            self._upper_set = True
            self._teaching  = False
            self._homed     = True
            self._node.get_logger().info(
                f'[sbc] UPPER limit set → {pos_mm:.1f}mm — teach-homing COMPLETE')
        return pos_mm

    def get_z_state(self, read_hw: bool = False) -> dict:
        """
        Returns cached Z state. Only reads hardware when read_hw=True
        (i.e. when a Z move is active). Prevents RS485 polling when idle.
        """
        if read_hw and self._z:
            try:
                raw = self._z.motor.read_encoder()
                self._z_pos_cache_mm = (raw - self._home_offset_counts) / self._z.counts_per_mm
            except Exception as e:
                self._node.get_logger().info(
                    f'[sbc] Z position read failed: {e}',
                    throttle_duration_sec=5.0)
        return {
            'position_m':   self._z_pos_cache_mm / 1000.0,
            'velocity_mps': 0.0,
            'moving':       self._homing,
            'fault':        False,
            'homed':        self._homed,
        }

    def get_wrist_deg(self) -> float:
        if not self._wrist:
            return 0.0
        return self._wrist.read_position_deg()

    # --- Spray / wrist ---
    def set_spray(self, on: bool):
        if self._gpio_ready:
            self._gpio.output(self._pin, self._gpio.HIGH if on else self._gpio.LOW)
        self._node.get_logger().debug(f'[sbc] spray → {"ON" if on else "OFF"}')

    def set_wrist(self, angle_deg: float):
        if self._wrist:
            self._wrist.move_to_deg(angle_deg)
            self._node.get_logger().info(f'[sbc] wrist cmd → {angle_deg:.1f}°',
                                         throttle_duration_sec=0.5)
        else:
            self._node.get_logger().warn(f'[sbc] wrist → {angle_deg:.1f}° (no driver)')

    # --- Base drive ---
    # def drive_base(self, linear_mps: float, angular_radps: float):
    #     if linear_mps >0.1:
    #         self._node.get_logger().info(
    #             f'[sbc] drive_base called — lin={linear_mps:.3f} ang={angular_radps:.3f}')
    #     if self._base:
    #         self._base.drive_velocity(-linear_mps, angular_radps)
    #     else:
    #         self._node.get_logger().warn('[sbc] drive_base — no BaseDrive instance')

    # def stop_base(self):
    #     self._node.get_logger().info('[sbc] stop_base called')
    #     if self._base:
    #         self._base.stop()
    #     else:
    #         self._node.get_logger().warn('[sbc] stop_base — no BaseDrive instance')

    def drive_base(self, linear_mps: float, angular_radps: float):
        if not self._base:
            self._node.get_logger().warn('[sbc] drive_base — no BaseDrive instance')
            return

        now    = time.time()
        key    = (round(linear_mps, 3), round(angular_radps, 3))
        moving = abs(linear_mps) > 0.005 or abs(angular_radps) > 0.005

        # Keepalive: moving commands refresh faster than stopped ones
        interval    = self._keepalive_moving_s if moving else self._keepalive_idle_s
        same_cmd    = (key == self._last_drive_key)
        resend_due  = (now - self._last_drive_send_t) > interval

        if same_cmd and not resend_due:
            return

        self._last_drive_key    = key
        self._last_drive_send_t = now

        if moving:
            self._node.get_logger().info(
                f'[sbc] drive_base — lin={linear_mps:.3f} ang={angular_radps:.3f}'
                f'{" (keepalive)" if same_cmd else ""}')

        # negative linear to fix mismatch in direction
        t0 = time.time()
        self._base.drive_velocity(-linear_mps, angular_radps)
        self._node.get_logger().debug(f'[timing] drive write {(time.time()-t0)*1000:.0f}ms',
                                     throttle_duration_sec=2.0)

    def stop_base(self):
        self._node.get_logger().info('[sbc] stop_base called')
        if self._base:
            self._base.stop()
            self._last_drive_key = None
            self._last_drive_send_t = time.time()
        else:
            self._node.get_logger().warn('[sbc] stop_base — no BaseDrive instance')

    def read_base_encoders(self) -> tuple:
        if self._base:
            try:
                l, r = self._base.read_encoders()
                return (l - self._enc_offset_l, r - self._enc_offset_r)
            except Exception as e:
                self._node.get_logger().info(
                    f'[sbc] encoder read failed: {e}',
                    throttle_duration_sec=5.0)
        return (0, 0)

    def spin_once(self):
        # No autonomous homing motion — homing is taught by the operator
        # via home_z()/teach_set_lower()/teach_set_upper(), not polled here.
        pass


# ---------------------------------------------------------------------------
# Bridge node
# ---------------------------------------------------------------------------
class ActuatorBridgeNode(Node):

    BACKENDS = {'sim': SimBackend, 'sbc': SBCBackend}

    def __init__(self):
        super().__init__('actuator_bridge_node')

        # --- Z-axis params ---
        self.declare_parameter('hardware_backend',  'sbc')
        self.declare_parameter('serial_port',       '/dev/ttyAMA0')
        self.declare_parameter('serial_baud',       115200)
        self.declare_parameter('z_slave_id',        3)
        self.declare_parameter('z_counts_per_mm',   100.0)
        self.declare_parameter('z_counts_per_rev',  10000)
        self.declare_parameter('z_travel_rpm',      200)
        self.declare_parameter('z_accel_up_ms',   150)
        self.declare_parameter('z_decel_up_ms',   120)
        self.declare_parameter('z_accel_down_ms', 180)
        self.declare_parameter('z_decel_down_ms', 100)
        self.declare_parameter('z_paint_speed_mps', 0.8)   # paint stroke speed (m/s)
        self.declare_parameter('z_max_m',           1.2)
        self.declare_parameter('z_sim_speed_mps',   0.8)
        self.declare_parameter('strip_z_max',       1.2)
        self.declare_parameter('solenoid_gpio_pin', 17)

        # --- Homing / soft-limit params ---
        # No physical limit switches — Z is homed manually by the operator
        # (joystick LB arms teach mode, A records lower/zero, Y records upper).
        # These are just the fallback defaults before the first teach.
        self.declare_parameter('z_soft_lower_mm',   0.0)
        self.declare_parameter('z_soft_upper_mm',   1200.0)

        # --- Base drive params ---
        self.declare_parameter('base_left_id',           1)
        self.declare_parameter('base_right_id',          2)
        self.declare_parameter('wheel_dia_m',            0.1524)
        self.declare_parameter('track_width_m',          0.7)
        self.declare_parameter('base_gear_ratio',        20.0)
        self.declare_parameter('base_counts_per_rev',    10000)
        self.declare_parameter('zero_encoders_on_start', True)  # safe default
        self.declare_parameter('enable_odom', True)
        self._enable_odom = self.get_parameter('enable_odom').value      # no underscore
        if not self._enable_odom:
            self.get_logger().info('[bridge] odom disabled — /paint_bot/odom will not publish')

        self.declare_parameter('keepalive_moving_s', 0.25)   # 2 Hz while driving
        self.declare_parameter('keepalive_idle_s',   2.0)   # 0.5 Hz when stopped    

        self.declare_parameter('cmd_vel_timeout_s', 0.3) # needed to ensure zero commands are send 

        #wrist params
        self.declare_parameter('wrist_port',          '/dev/ttyACM2')  # TBD via udev symlink
        self.declare_parameter('wrist_servo_id',      1)               # TBD
        self.declare_parameter('wrist_center_pos',    828)            # TBD
        self.declare_parameter('wrist_counts_per_deg',11.375)          # TBD but datasheet typical ST series (4096 counts / 360°)
        self.declare_parameter('wrist_min_pos',       0)            # TBD
        self.declare_parameter('wrist_max_pos',       4095)            # TBD

        # --- Odom frame params ---
        self.declare_parameter('odom_frame_id',  'odom')
        self.declare_parameter('base_frame_id',  'base_link')

        # --- Backend ---
        backend_name = self.get_parameter('hardware_backend').value
        cls = self.BACKENDS.get(backend_name)
        if cls is None:
            raise ValueError(f'Unknown backend: {backend_name}')

        self._backend = cls(self)
        self.get_logger().info(
            f'actuator_bridge_node started — backend={backend_name}')

        # --- Z move active flag — controls hardware polling in _loop ---
        self._z_move_active = False

        # Single motion-stop state, driven solely by /safety_command tier:
        #   None     — running, drive commands accepted
        #   'HOLD'   — stopped + inhibited, recoverable, limits kept
        #   'FAULT'  — stopped + inhibited, recoverable (queue flush is the FSM's job), limits kept
        #   'ESTOP'  — stopped + inhibited, gantry limits/homing cleared
        # Any non-None value blocks /z_axis_cmd and /cmd_vel. This replaces the old
        # _faulted boolean and the /robot_state-driven stop. /safety_command is the
        # authority and the only tier source — it alone distinguishes the three.
        # NOTE: FSM-direct FAULTs that don't route through the safety node (e.g.
        # CORNER_ABORT, handler-returned FAULT) will NOT latch this inhibit until
        # they are migrated to /fault_report. Accepted tradeoff — see migration note.
        self._motion_stop = None

        # --- Subscriptions ---
        self.create_subscription(ZAxisCmd, '/z_axis_cmd',  self._z_cmd_cb,    5)
        self.create_subscription(WristCmd, '/wrist_cmd',   self._wrist_cb,    5)
        self.create_subscription(SprayCmd, '/spray_cmd',   self._spray_cb,    10)
        self.create_subscription(
            Twist, '/paint_bot/cmd_vel', self._base_cmd_cb, 1)
        self.create_subscription(GantryLimitCmd, '/gantry_limit_cmd', self._gantry_limit_cb, 10)

        # /safety_command is the single authority for the stop state (tier + reason).
        self.create_subscription(SafetyCommand, '/safety_command', self._safety_cmd_cb, 10)

        # --- Soft Z travel limits (per-environment; enforced only once homed) ---
        self._z_soft_lower_mm = float(self.get_parameter('z_soft_lower_mm').value)
        self._z_soft_upper_mm = float(self.get_parameter('z_soft_upper_mm').value)

        # --- Publishers ---
        self._state_pub      = self.create_publisher(PaintHeadStatus, '/paint_head_status', 10)

        self._odom_pub = None
        if self._enable_odom:
            self._odom_pub = self.create_publisher(Odometry, '/paint_bot/odom', 7)
        else:
            self.get_logger().info('[bridge] odom disabled — /paint_bot/odom will not publish')

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Odom state ---
        self._odom_x         = 0.0
        self._odom_y         = 0.0
        self._odom_th        = 0.0
        self._last_enc_l     = 0
        self._last_enc_r     = 0
        self._last_odom_time = self.get_clock().now()

        # --- Timers ---
        self.create_timer(0.02, self._loop)       # 50 Hz — paint head status
        self.create_timer(0.1,  self._odom_loop)  # 5Hz Hz — odometry

        self._cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout_s').value)
        self._last_cmd_vel_time = None      # None = never received, don't watchdog
        self._base_stopped_by_wd = False

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    # Map SafetyCommand tier -> motion-stop value. NOMINAL/SAFETY_HOLD-absent -> None.
    _TIER_TO_STOP = {
        SafetyCommand.SAFETY_HOLD: 'HOLD',
        SafetyCommand.FAULT:       'FAULT',
        SafetyCommand.ESTOP:       'ESTOP',
    }

    def _safety_cmd_cb(self, msg: SafetyCommand):
        # Single authority for the actuator stop state. On the EDGE into any stop
        # tier (HOLD/FAULT/ESTOP) we kill all actuators once; while held, the Z/base
        # callbacks refuse commands. On return to NOMINAL we release. ESTOP alone
        # clears gantry limits/homing. One zero per edge — no flooding.
        new_stop = self._TIER_TO_STOP.get(msg.tier)   # None for NOMINAL
        prev = self._motion_stop
        self._motion_stop = new_stop

        if new_stop and new_stop != prev:
            # entering a stop tier (or stepping HOLD->FAULT->ESTOP) — one-shot kill
            self.get_logger().error(f'[bridge] {new_stop} ({msg.reason}) — killing all actuators')
            for label, fn in (('base stop', lambda: self._backend.drive_base(0.0, 0.0)),
                              ('z stop',    self._backend.stop_z),
                              ('spray off', lambda: self._backend.set_spray(False))):
                try:
                    fn()
                except Exception as e:
                    self.get_logger().error(f'[bridge] {label} failed: {e}')
            self._z_move_active = False
            # ESTOP is the ONLY tier that clears gantry limits/homing (re-teach).
            # HOLD/FAULT preserve them — a recoverable stop must not force a re-teach.
            if new_stop == 'ESTOP':
                self.get_logger().warn('[bridge] ESTOP — clearing gantry limits (re-teach required)')
                try:
                    self._backend.clear_limits()
                except Exception as e:
                    self.get_logger().error(f'[bridge] clear_limits failed: {e}')
            else:
                self.get_logger().info(f'[bridge] {new_stop} — gantry limits/homing preserved')

        elif new_stop is None and prev is not None:
            self.get_logger().info(f'[bridge] stop released ({prev} → NOMINAL) — actuators re-enabled')

    # def _robot_state_cb(self, msg: String):
    #     entering_fault = (msg.data == 'FAULT' and self._prev_robot_state != 'FAULT')
    #     self._faulted = (msg.data == 'FAULT')

    #     if entering_fault:
    #         self.get_logger().error('[bridge] FAULT — killing all actuators')
    #         for label, fn in (('base stop',  lambda: self._backend.drive_base(0.0, 0.0)),
    #                           ('z stop',     self._backend.stop_z),
    #                           ('spray off',  lambda: self._backend.set_spray(False))):
    #             try:
    #                 fn()
    #             except Exception as e:
    #                 self.get_logger().error(f'[bridge] {label} failed: {e}')
    #         self._z_move_active = False

    #     self._prev_robot_state = msg.data   # always update — re-arms the edge

    def _paint_mps_to_rpm(self, v_mps: float) -> int:
        """Convert a linear Z speed (m/s) to motor-shaft RPM for command_position.

        counts_per_mm is post-gearbox (output); z_counts_per_rev is motor-shaft
        (pre-gearbox). Their ratio already includes the gearing, so:
            motor_rev_per_m = counts_per_mm * 1000 / counts_per_rev
            rpm             = v_mps * motor_rev_per_m * 60
        e.g. 0.6 m/s * (576.03*1000/10000) * 60 ≈ 2074 rpm.
        """
        counts_per_mm  = float(self.get_parameter('z_counts_per_mm').value)
        counts_per_rev = float(self.get_parameter('z_counts_per_rev').value) \
            if self.has_parameter('z_counts_per_rev') else 10000.0
        motor_rev_per_m = counts_per_mm * 1000.0 / counts_per_rev
        return int(v_mps * motor_rev_per_m * 60.0)

    def _z_cmd_cb(self, msg: ZAxisCmd):
        if self._motion_stop:
            return    # HOLD/FAULT/ESTOP — motion inhibited, ignore all Z commands

        homed = getattr(self._backend, '_homed', False)
        self.get_logger().info(
            f'[bridge] z_axis_cmd z_mode={msg.z_mode} target={msg.z_target_pos_mm:.1f}mm '
            f'rpm={msg.rpm} vel={msg.z_velocity} homed={homed}')

        if msg.z_mode == ZAxisCmd.Z_HOME:
            # Arms manual teach-homing — no autonomous motion, so no need
            # to force hardware polling on. Jog (Z_VEL) drives the motor.
            self._backend.home_z()

        elif msg.z_mode == ZAxisCmd.Z_POSITION:
            target = msg.z_target_pos_mm
            if homed:   # clamp only once the zero datum is meaningful
                target = max(self._z_soft_lower_mm, min(self._z_soft_upper_mm, target))
                if target != msg.z_target_pos_mm:
                    self.get_logger().warn(
                        f'[bridge] target clamped {msg.z_target_pos_mm:.1f}→{target:.1f}mm (soft limits)')
            if msg.rpm > 0:
                rpm = int(msg.rpm)                       # explicit override from command
            else:
                paint_mps = float(self.get_parameter('z_paint_speed_mps').value)
                rpm = self._paint_mps_to_rpm(paint_mps)  # paint speed (m/s) → motor rpm
            self._z_move_active = True
            self._backend.move_z(target / 1000.0, rpm)

        elif msg.z_mode == ZAxisCmd.Z_VEL:
            vel = int(msg.z_velocity)
            teaching = getattr(self._backend, '_teaching', False)
            if not homed and not teaching:
                self.get_logger().warn('[bridge] jog blocked — press LB to arm homing first')
                vel = 0
            elif vel != 0 and homed:
                # Bound clamp only means anything once the zero datum
                # is established. During teaching (pre-homing) jog is
                # unclamped — operator watches the gantry manually, same
                # as they would with a physical limit switch missing.
                # Gated entirely by the taught soft limits — no separate
                # hard-coded envelope; the teach-homing range IS the limit.
                pos_mm = self._backend.get_z_state(read_hw=True)['position_m'] * 1000.0
                if vel < 0 and pos_mm <= self._z_soft_lower_mm:
                    self.get_logger().warn(
                        f'[bridge] down-jog blocked — taught lower limit {self._z_soft_lower_mm:.0f}mm')
                    vel = 0
                elif vel > 0 and pos_mm >= self._z_soft_upper_mm:
                    self.get_logger().warn(
                        f'[bridge] up-jog blocked — taught upper limit {self._z_soft_upper_mm:.0f}mm')
                    vel = 0
            self._z_move_active = (vel != 0)
            self._backend.jog_z(vel)

        elif msg.z_mode == ZAxisCmd.Z_STOP:
            self._z_move_active = False
            self._backend.stop_z()

        else:
            self.get_logger().warn(f'[bridge] Unknown z_mode={msg.z_mode}; stopping Z')
            self._z_move_active = False
            self._backend.stop_z()

    def _gantry_limit_cb(self, msg: GantryLimitCmd):
        """
        Teaches Z travel limits — this IS how homing completes on this
        machine (no physical limit switches). set_lower (A) either defines
        the zero datum (first time, pre-homing) or re-teaches the soft
        lower bound (post-homing); set_upper (Y) either finishes homing
        (first time) or re-teaches the soft upper bound (post-homing).
        """
        teaching = getattr(self._backend, '_teaching', False)
        homed    = getattr(self._backend, '_homed', False)
        if not teaching and not homed:
            self.get_logger().warn(
                '[bridge] gantry limit ignored — press LB to arm homing first')
            return

        if msg.set_lower:
            pos_mm = self._backend.teach_set_lower()
            if pos_mm is not None:
                self._z_soft_lower_mm = pos_mm
                self.get_logger().info(
                    f'[bridge] Z LOWER limit set → {pos_mm:.1f}mm'
                    + ('' if homed else ' (zero datum)'))

        if msg.set_upper:
            pos_mm = self._backend.teach_set_upper()
            if pos_mm is not None:
                self._z_soft_upper_mm = pos_mm
                self.get_logger().info(
                    f'[bridge] Z UPPER limit set → {pos_mm:.1f}mm'
                    + ('' if homed else ' — teach-homing COMPLETE'))

    def _wrist_cb(self, msg: WristCmd):
        self._backend.set_wrist(msg.wrist_target_deg)

    def _spray_cb(self, msg: SprayCmd):
        self._backend.set_spray(msg.valve_open)

    def _base_cmd_cb(self, msg: Twist):
        if self._motion_stop:
            return    # HOLD/FAULT/ESTOP — motion inhibited, ignore all base commands
        self._backend.drive_base(msg.linear.x, msg.angular.z)

        self._last_cmd_vel_time = self.get_clock().now()
        self._base_stopped_by_wd = False

    # -------------------------------------------------------------------------
    # Paint head status loop — 50 Hz
    # Only reads Z hardware when _z_move_active — prevents RS485 blocking when idle
    # -------------------------------------------------------------------------
    def _loop(self):
        self._backend.spin_once()

        s   = self._backend.get_z_state(read_hw=self._z_move_active)

        # if self._last_cmd_vel_time is not None and not self._base_stopped_by_wd:
        #     age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds * 1e-9
        #     if age > self._cmd_vel_timeout:
        #         self._backend.stop_base()
        #         self._base_stopped_by_wd = True
        #         self.get_logger().warn(
        #             f'[bridge] cmd_vel watchdog — no command for {age:.2f}s, base stopped')

        # Auto-stop any active Z move (jog or commanded stroke) once it
        # reaches a taught soft limit (homed only) — no separate hard-coded
        # envelope; the teach-homing range IS the limit.
        if getattr(self._backend, '_homed', False) and self._z_move_active:
            pos_mm = s['position_m'] * 1000.0
            target_mm = getattr(self._backend, '_z_target', pos_mm / 1000.0) * 1000.0
            jog_dir = getattr(self._backend, '_jog_dir', 0)

            moving_down = (jog_dir < 0) or (target_mm < pos_mm - 1.0)
            moving_up   = (jog_dir > 0) or (target_mm > pos_mm + 1.0)

            if (pos_mm <= self._z_soft_lower_mm and moving_down) or \
               (pos_mm >= self._z_soft_upper_mm and moving_up):
                self._backend.stop_z()
                self._z_move_active = False
                self.get_logger().warn(
                    f'[bridge] Z auto-stopped at taught limit ({pos_mm:.1f}mm)')

        # No physical limit switches on this machine — limit_lower_z carries
        # the "homed" signal (teach-homing complete) that paint_executor_node
        # and _start_stroke() gate on; limit_upper_z has no hardware source.
        pub = PaintHeadStatus()
        pub.header.stamp     = self.get_clock().now().to_msg()
        pub.z_actual_pos_mm  = float(s['position_m'] * 1000.0)
        pub.limit_lower_z    = bool(s['homed'])
        pub.limit_upper_z    = False
        pub.z_soft_lower_mm  = float(self._z_soft_lower_mm)
        pub.z_soft_upper_mm  = float(self._z_soft_upper_mm)
        # Wrist is on its own serial port; poll at ~5 Hz so it doesn't slow the 50 Hz loop (in sim update immediately)
        if isinstance(self._backend, SimBackend):
            self._wrist_actual_deg = self._backend.get_wrist_deg()
        else:
            self._wrist_poll = getattr(self, '_wrist_poll', 0) + 1
            if self._wrist_poll % 10 == 0:
                self._wrist_actual_deg = self._backend.get_wrist_deg()
                self.get_logger().debug(
                    f'[wrist] actual={self._wrist_actual_deg:.1f}°',
                    throttle_duration_sec=0.5)
        pub.wrist_actual_deg = float(getattr(self, '_wrist_actual_deg', 0.0))
        pub.paint_level      = 1.0
        self._state_pub.publish(pub)

    # -------------------------------------------------------------------------
    # Odometry loop — 10 Hz
    # Continuous but guarded — failed encoder reads log and return (0,0)
    # -------------------------------------------------------------------------


    

    def _odom_loop(self):
        if not self._enable_odom:
         return
    
        now = self.get_clock().now()
        dt  = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now

        if dt <= 0.0:
            return

        t0 = time.time()
        enc_l, enc_r = self._backend.read_base_encoders()

        self.get_logger().debug(f'[timing] enc read {(time.time()-t0)*1000:.0f}ms',
                                       throttle_duration_sec=2.0)

        wheel_dia   = self.get_parameter('wheel_dia_m').value
        gear_ratio  = self.get_parameter('base_gear_ratio').value
        cpr         = self.get_parameter('base_counts_per_rev').value
        track_width = self.get_parameter('track_width_m').value

        counts_per_m = (cpr * gear_ratio) / (math.pi * wheel_dia)

        delta_l = enc_l - self._last_enc_l
        delta_r = enc_r - self._last_enc_r

        # get distance traveled by left and right, with differential drive correction
        dl = -delta_l / counts_per_m # add correction for differential drive 
        dr =  delta_r / counts_per_m

        self._last_enc_l = enc_l
        self._last_enc_r = enc_r

        

        # self.get_logger().debug(
        # f'[odom] enc_l={enc_l} enc_r={enc_r} '
        # f'dl={dl:.4f}m dr={dr:.4f}m '
        # f'counts_per_m={counts_per_m:.1f} '
        # f'cpr={cpr} gear={gear_ratio} dia={wheel_dia} track={track_width}',
        # throttle_duration_sec=3.0)

        # Differential drive kinematics
        d_center = (dl + dr) / 2.0 #  just avg translation..
        d_theta  = (dl - dr) / track_width # difference of arc lengths traveled by left and right wheels / diff of radii (which is trackwidth)

        # in odom frame, +x, +y, origin initislly align at boot
        # get baselink frame in odom frame using [x,y] = [d_center*cos(theta),d_center*sin(theta)]
        self._odom_th += d_theta
        self._odom_th = math.atan2(math.sin(self._odom_th), math.cos(self._odom_th))  # wrap to [-π, π]
        self._odom_x  += d_center * math.cos(self._odom_th)
        self._odom_y  += d_center * math.sin(self._odom_th)


        # get velocity
        vx  = d_center / dt
        vth = d_theta  / dt

        self.get_logger().debug(
        f'[odom] x={self._odom_x:.3f} y={self._odom_y:.3f} '
        f'th={math.degrees(self._odom_th):.1f}° vx={vx:.3f} vth={vth:.3f}',
        throttle_duration_sec=3.0)

        # Quaternion from yaw
        q_z = math.sin(self._odom_th / 2.0)
        q_w = math.cos(self._odom_th / 2.0)

        odom_frame = self.get_parameter('odom_frame_id').value
        base_frame = self.get_parameter('base_frame_id').value

        # TF broadcast odom → base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now.to_msg()
        tf_msg.header.frame_id = odom_frame
        tf_msg.child_frame_id  = base_frame
        tf_msg.transform.translation.x = self._odom_x
        tf_msg.transform.translation.y = self._odom_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z    = q_z
        tf_msg.transform.rotation.w    = q_w
        self._tf_broadcaster.sendTransform(tf_msg)

        # Odom message
        odom = Odometry()
        odom.header.stamp              = now.to_msg()
        odom.header.frame_id           = odom_frame
        odom.child_frame_id            = base_frame
        odom.pose.pose.position.x      = self._odom_x
        odom.pose.pose.position.y      = self._odom_y
        odom.pose.pose.orientation.z   = q_z
        odom.pose.pose.orientation.w   = q_w
        odom.twist.twist.linear.x      = vx
        odom.twist.twist.angular.z     = vth
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()