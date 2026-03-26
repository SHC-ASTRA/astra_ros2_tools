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


class FeedbackGenerator_Arm(Node):
    """Generate feedback messages as if they were coming from the rover over VicCAN."""

    def __init__(self):
        super().__init__("feedback_generator_arm_node")

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
        # Arm encoder angles
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="arm",
                command_id=55,  # ARM ENCODERS
                data=[100, 200, 300, 400],
            )
        )

        # Arm joint velocities
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="arm",
                command_id=59,  # ARM JOINT VELOCITIES
                data=[1000, 2000, 3000, 4000],
            )
        )

    def voltage_feedback_timer_callback(self):
        # Socket voltage (batt, 12, 5, 3)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="arm",
                command_id=54,  # VOLTAGE
                data=[1460, 1250, 490, 310],
            )
        )

        # Digit voltage (batt, 12, 5)
        self.send_viccan(
            VicCAN(
                header=Header(),
                mcu_name="digit",
                command_id=54,  # VOLTAGE
                data=[1460, 1250, 490, 0],
            )
        )

    def rev_feedback_timer_callback(self):
        for i in range(4):
            # REV Motor Power (ID, temp, voltage, current)
            self.send_viccan(
                VicCAN(
                    header=Header(),
                    mcu_name="arm",
                    command_id=53,  # REV MOTOR
                    data=[i + 1, 300, 149, 100],
                )
            )
            # REV Motor Motion (ID, pos, vel)
            self.send_viccan(
                VicCAN(
                    header=Header(),
                    mcu_name="arm",
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

    feedback_node = FeedbackGenerator_Arm()

    try:
        rclpy.spin(feedback_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
