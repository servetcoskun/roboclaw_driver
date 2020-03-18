#!/usr/bin/env python

from __future__ import print_function
from math import pi, cos, sin

import traceback
import threading

import rospy
import diagnostic_updater
import diagnostic_msgs
import tf

from roboclaw_driver.msg import Stats, SpeedCmd
from roboclaw_driver import RoboclawControl, Roboclaw, RoboclawStub

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


DEFAULT_DEV_NAMES = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw"
DEFAULT_LOOP_HZ = 10
DEFAULT_ADDRESS = 0x80
DEFAULT_DEADMAN_SEC = 3
DEFAULT_STATS_TOPIC = "~stats"
DEFAULT_SPEED_CMD_TOPIC = "~speed_command"
DEFAULT_ODOM_TOPIC = "~odom"


class RoboclawNode:
    def __init__(self, node_name):
        """
        Parameters:
            :str node_name: ROS node name
            :RoboclawControl rbc_ctl: RoboclawControl obj. controlling hardware
            :int loop_rate: Integer rate in Hz of the main loop
        """
        self._node_name = node_name
        self._rbc_ctls = []  # Populated by the connect() method

        # Records the values of the last speed command
        self._p_cmd_time = rospy.get_rostime()
        self._p_cmd_m1_qpps = 0
        self._p_cmd_m2_qpps = 0
        self._p_cmd_accel = 0
        self._p_cmd_max_secs = 0

        # Serialize access to cmd variables
        self._speed_cmd_lock = threading.RLock()

        # Setup encoder parameters
        self._ticks_pr_meter = 4342.2
        self._base_width = 0.315
        self._p_enc_left = 0
        self._p_enc_right = 0
        self._p_enc_time = rospy.get_rostime()

        # Setup position parameters
        self._x = 0
        self._y = 0
        self._th = 0.0

        # Set up the Publishers
        self._stats_pub = rospy.Publisher(
            rospy.get_param("~stats_topic", DEFAULT_STATS_TOPIC),
            Stats,
            queue_size=1
        )

        # Set up odometry publisher
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Set up odometry-tf broadcaster
        self._odom_broadcaster = tf.TransformBroadcaster()

        # Set up the Diagnostic Updater
        self._diag_updater = diagnostic_updater.Updater()
        self._diag_updater.setHardwareID(node_name)
        self._diag_updater.add("Read Diagnostics", self._publish_diagnostics)

        # Set up the SpeedCmd Subscriber
        rospy.Subscriber(
            rospy.get_param("~speed_cmd_topic", DEFAULT_SPEED_CMD_TOPIC),
            SpeedCmd,
            self._speed_cmd_callback
        )

        # For logdebug
        self.prev_m1_val = 0
        self.prev_m2_val = 0

    @property
    def roboclaw_control(self):
        return self._rbc_ctl

    def connect(self, dev_name, baud_rate, address, test_mode=False):
        """Connects the node to the Roboclaw controller, or the test stub
        Parameters:
            :param str dev_name: Serial device name (e.g. /dev/ttyACM0)
            :param int baud_rate: Serial baud rate (e.g. 115200)
            :param int address: Serial address (default 0x80)
            :param bool test_mode: if connecting to the controller test stub
        """
        rospy.loginfo("Connecting to roboclaw")
        if not test_mode:
            roboclaw = Roboclaw(dev_name, baud_rate)
        else:
            roboclaw = RoboclawStub(dev_name, baud_rate)
        self._rbc_ctls.append(RoboclawControl(roboclaw, address))

    def run(self, loop_hz=10, deadman_secs=1):
        """Runs the main loop of the node
        Parameters:
            :param int loop_hz: Loops per sec of main loop (default 10 hertz)
            :param int deadman_secs: time before Roboclaw will cont. last cmd
                before stopping if another command is not received
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(loop_hz)

        try:
            rospy.loginfo("Starting main loop")

            while not rospy.is_shutdown():
                # ros time for later use
                rtime = rospy.get_rostime()
                # Read and publish encoder readings
                read_success, stats = self._rbc_ctls[0].read_stats()
                for error in stats.error_messages:
                    rospy.logwarn(error)
                if read_success:
                    self._publish_stats(stats)
                else:
                    rospy.logwarn(
                        "Error reading stats from Roboclaw: {}".format(stats))

                # Stop motors if running and no commands are being received
                if (stats.m1_enc_qpps != 0 or stats.m2_enc_qpps != 0):
                    if (rtime - self._p_cmd_time).to_sec() > deadman_secs:
                        rospy.loginfo(
                            "No cmd for over 1 sec: Stopping motors")
                        m1_enc_qpps_abs = abs(stats.m1_enc_qpps)
                        m2_enc_qpps_abs = abs(stats.m2_enc_qpps)
                        decel = max(m1_enc_qpps_abs, m2_enc_qpps_abs) * 2
                        for rbc_ctl in self._rbc_ctls:
                            rbc_ctl.stop(decel=decel)

                # Publish diagnostics
                self._diag_updater.update()
                # Update
                enc_left = stats.m1_enc_val
                enc_right = stats.m2_enc_val
                self._publish_update(enc_left, enc_right)

                # Publish odometry data
                # self._publish_odom(0,0,0,0,0,0)

                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self._p_enc_left
        right_ticks = enc_right - self._p_enc_right

        self._p_enc_left = enc_left
        self._p_enc_right = enc_right

        dist_l = left_ticks / self._ticks_pr_meter
        dist_r = right_ticks / self._ticks_pr_meter
        dist = (dist_l + dist_r) / 2.0

        t = rospy.get_rostime()
        dt = (t - self._p_enc_time).to_sec()
        self._p_enc_time = t

        if left_ticks == right_ticks:
            dist_th = 0.0
            self._x += dist * cos(self._th)
            self._y += dist * sin(self._th)
        else:
            dist_th = (dist_r - dist_l) / self._base_width
            r = dist / dist_th
            self._x += r * (sin(dist_th + self._th) - sin(self._th))
            self._y -= r * (cos(dist_th + self._th) - cos(self._th))
            self._th = self.normalize_angle(self._th + dist_th)

        if abs(dt) < 0.000001:
            vel_x = 0.0
            vel_th = 0.0
        else:
            vel_x = dist / dt
            vel_th = dist_th / dt

        return vel_x, vel_th

    def _publish_update(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self._p_enc_left) > 20000:
            rospy.logerr(
                "Ignoring left enc. jump: cur %d, prev. %d"
                % (enc_left, self._p_enc_left))
        elif abs(enc_right - self._p_enc_right) > 20000:
            rospy.logerr(
                "Ignoring right enc. jump: cur %d, prev. %d"
                % (enc_right, self._p_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self._publish_odom(self._x, self._y, self._th, vel_x, vel_theta)

    def _publish_odom(self, x, y, th, vx, vth):
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "odom"

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, -vth)

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        # publish the transform over tf
        self._odom_broadcaster.sendTransform(
            (x, y, 0),
            odom_quat,
            rospy.get_rostime(),
            "base_link",
            "odom"
        )
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
        odom.twist.covariance = odom.pose.covariance

        # publish the message
        self._odom_pub.publish(odom)

    def _publish_stats(self, stats):
        """Publish stats to the <node_name>/stats topic

        Parameters:
            :param roboclaw_control.RoboclawStats stats: Stats from
            the Roboclaw controller
        """
        msg = Stats()
        msg.header.stamp = rospy.get_rostime()

        msg.m1_enc_val = stats.m1_enc_val
        msg.m1_enc_qpps = stats.m1_enc_qpps

        msg.m2_enc_val = stats.m2_enc_val
        msg.m2_enc_qpps = stats.m2_enc_qpps

        # rospy.logdebug("Encoder diffs M1:{}, M2:{}".format(
        #     stats.m1_enc_val - self.prev_m1_val,
        #     stats.m2_enc_val - self.prev_m2_val
        # ))
        self.prev_m1_val = stats.m1_enc_val
        self.prev_m2_val = stats.m2_enc_val

        self._stats_pub.publish(msg)

    def _publish_diagnostics(self, stat):
        """Function called by the diagnostic_updater to fetch and publish diagnostics
        from the Roboclaw controller

        Parameters:
        :param diagnostic_updater.DiagnosticStatusWrapper stat:
            DiagnosticStatusWrapper provided by diagnostic_updater when called

        Returns: The updated DiagnosticStatusWrapper
        :rtype: diagnostic_updater.DiagnosticStatusWrapper
        """
        for i, rbc_ctl in enumerate(self._rbc_ctls):
            diag = rbc_ctl.read_diag()

            stat.add("[{}] Temperature 1 (C):".format(i), diag.temp1)
            stat.add("[{}] Temperature 2 (C):".format(i), diag.temp2)
            stat.add("[{}] Main Battery (V):".format(i), diag.main_battery_v)
            stat.add("[{}] Logic Battery (V):".format(i), diag.logic_battery_v)
            stat.add("[{}] Motor 1 current (Amps):".format(i), diag.m1_current)
            stat.add("[{}] Motor 2 current (Amps):".format(i), diag.m2_current)

            for msg in diag.error_messages:
                level = diagnostic_msgs.msg.DiagnosticStatus.WARN
                if "error" in msg:
                    level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
                stat.summary(level, "[{}]: msg".format(i))

        return stat

    def _speed_cmd_callback(self, cmd):
        """
        Parameters:
            :param Speedcmd cmd: The forward/turn cmd message
        """
        with self._speed_cmd_lock:
            self._p_cmd_time = rospy.get_rostime()

            # Skip if the new cmd is not different than the last cmd
            if (cmd.m1_qpps == self._p_cmd_m1_qpps
                    and cmd.m2_qpps == self._p_cmd_m2_qpps
                    and cmd.accel == self._p_cmd_accel
                    and cmd.max_secs == self._p_cmd_max_secs):

                rospy.logdebug(
                    "Got Speed Cmd, but no change in cmd values")

            else:
                rospy.logdebug(
                    "M1 speed: {} | M2 speed: {} | Accel: {} | Max Secs: {}"
                    .format(cmd.m1_qpps, cmd.m2_qpps, cmd.accel, cmd.max_secs)
                )

                for rbc_ctl in self._rbc_ctls:
                    success = rbc_ctl.driveM1M2qpps(
                        cmd.m1_qpps, cmd.m2_qpps,
                        cmd.accel, cmd.max_secs
                    )

                    if not success:
                        rospy.logerr(
                            "RoboclawControl SpeedAccelDistanceM1M2 failed"
                            )

    def shutdown_node(self):
        """Performs Node shutdown tasks
        """
        rospy.loginfo("Shutting down...")


# --------------------------------------------------------------------------------

if __name__ == "__main__":

    # Setup the ROS node
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    print("node_name: {}".format(node_name))

    # Read the input parameters
    dev_names = rospy.get_param("~dev_names", DEFAULT_DEV_NAMES)
    baud_rate = int(rospy.get_param("~baud", DEFAULT_BAUD_RATE))
    address = int(rospy.get_param("~address", DEFAULT_ADDRESS))
    loop_hz = int(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ))
    deadman_secs = int(rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SEC))
    test_mode = bool(rospy.get_param("~test_mode", False))

    rospy.logdebug("node_name: {}".format(node_name))
    rospy.logdebug("dev_names: {}".format(dev_names))
    rospy.logdebug("baud: {}".format(baud_rate))
    rospy.logdebug("address: {}".format(address))
    rospy.logdebug("loop_hz: {}".format(loop_hz))
    rospy.logdebug("deadman_secs: {}".format(deadman_secs))
    rospy.logdebug("test_mode: {}".format(test_mode))

    node = RoboclawNode(node_name)
    rospy.on_shutdown(node.shutdown_node)

    try:
        # Initialize the Roboclaw controllers
        for dev in dev_names.split(','):
            node.connect(dev, baud_rate, address, test_mode)
        node.run(loop_hz=loop_hz, deadman_secs=DEFAULT_DEADMAN_SEC)

    except Exception as e:
        rospy.logfatal(
            "Unhandled exeption...printing stack trace then shutting down node"
            )
        rospy.logfatal(
            traceback.format_exc()
            )

    # Shutdown and cleanup
    if node:
        node.shutdown_node()
    rospy.loginfo("Shutdown complete")
