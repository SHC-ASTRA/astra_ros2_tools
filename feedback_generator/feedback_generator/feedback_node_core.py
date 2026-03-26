import sys
import threading
import signal
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy import qos

from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState, Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, Twist
from control_msgs.msg import JointJog
from astra_msgs.msg import (
    SocketFeedback,
    DigitFeedback,
    ArmManual,
    ArmFeedback,
    VicCAN,
    RevMotorState,
    CoreControl,
    CoreFeedback,
    RevMotorState,
    BioControl,
    BioFeedback,
    NewCoreFeedback,
    Barometer,
    CoreCtrlState,
)

control_qos = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,  # Best Effort subscribers are still compatible with Reliable publishers
    durability=qos.QoSDurabilityPolicy.VOLATILE,
    # deadline=Duration(seconds=1),
    # lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    # liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    # liveliness_lease_duration=Duration(seconds=5),
)


class FeedbackGenerator_Core(Node):
    """Generate feedback messages as if they were coming from the rover over VicCAN."""

    def __init__(self):
        super().__init__("feedback_generator_core_node")

        ##################################################
        # Parameters

        self.declare_parameter("listen_commands", True)
        self.listen_commands = (
            self.get_parameter("listen_commands").get_parameter_value().bool_value
        )

        ###################################################
        # Topics

        # Anchor topics

        # from_vic
        self.anchor_fromvic_pub_ = self.create_publisher(
            String, "/anchor/from_vic/mock_mcu", 20
        )

        ###################################################
        # Timers

        self.main_feedback_timer_ = self.create_timer(
            2.0, self.main_feedback_timer_callback
        )

        self.voltage_feedback_timer_ = self.create_timer(
            1.0, self.voltage_feedback_timer_callback
        )

        self.rev_feedback_timer_ = self.create_timer(
            0.5, self.rev_feedback_timer_callback
        )

    def main_feedback_timer_callback(self):
        # GPS (lat)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=48,  # GPS LAT
                data=[35.54125425],
            )
        )
        # GPS (lon)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=49,  # GPS LON
                data=[-120.2524265412],
            )
        )
        # GPS (sat, alt)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=50,  # GPS SAT/ALT
                data=[30.0, 110.53],
            )
        )

        # IMU (Gyro, calib)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=51,  # IMU GYRO/CALIB
                data=[1, 2, 3, 3313],
            )
        )
        # IMU (Accel, heading)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=52,  # IMU ACCEL/HEADING
                data=[1, 2, 3, 45],
            )
        )

        # BMP (Hum, alt, pressure)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=56,  # BMP HUM/ALT/PRESSURE
                data=[65, 100, 1013, 0],
            )
        )

    def voltage_feedback_timer_callback(self):
        # Voltage (batt, 12, 5, 3)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="core",
                command_id=54,  # VOLTAGE MAIN/AUX
                data=[1460, 1250, 490, 310],
            )
        )

    def rev_feedback_timer_callback(self):
        for i in range(4):
            # REV Motor Power (ID, temp, voltage, current)
            self.send_viccan(
                VicCAN(
                    header=Header(),
                    mcu_name="core",
                    command_id=53,  # REV MOTOR
                    data=[i + 1, 300, 149, 100],
                )
            )
            # REV Motor Motion (ID, pos, vel)
            self.send_viccan(
                VicCAN(
                    header=Header(),
                    mcu_name="core",
                    command_id=58,  # REV MOTOR MOTION
                    data=[i + 1, 1000, 200, 0],
                )
            )

    def send_viccan(self, msg: VicCAN):
        """For now, send messages as strings on /anchor/from_vic/mock_mcu"""
        data = f"can_relay_fromvic,{msg.mcu_name},{msg.command_id}," + ",".join(
            str(x) for x in msg.data
        )
        self.anchor_fromvic_pub_.publish(String(data=data))


def exit_handler(signum, frame):
    print("Caught SIGTERM. Exiting...")
    rclpy.try_shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    # Catch termination signals and exit cleanly
    signal.signal(signal.SIGTERM, exit_handler)

    feedback_node = FeedbackGenerator_Core()

    try:
        rclpy.spin(feedback_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
