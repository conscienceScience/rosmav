#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from time import sleep


class DanceNode(Node):
    def __init__(self):
        super().__init__("dancing_node")

        self._step_counter = 0
        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.loop = self.create_timer(1.0, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 8
        self.command_pub.publish(neutral)

    def _loop(self):
        # Assuming _step_counter is managed elsewhere, otherwise adjust logic accordingly
        if self._step_counter > 60:
            self.destroy_node()
            return
        self._dance_moves()

    def _dance_moves(self):
        # Define your dance moves here based on your channel configurations
        pass

    def move_forward(self, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1500, 1500, 1900, 1500, 1500, 1500]
        self.publish_and_log(msg, "Moving forward")

    def move_left(self, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1500, 1500, 1500, 1900, 1500, 1500]
        self.publish_and_log(msg, "Moving left")

    def move_right(self, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1500, 1500, 1500, 1100, 1500, 1500]
        self.publish_and_log(msg, "Moving right")

    def up(self, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1900, 1500, 1700, 1500, 1500, 1500]
        self.publish_and_log(msg, "Rising")

    def down(self, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1100, 1500, 1100, 1500, 1500, 1500]
        self.publish_and_log(msg, "Dropping")

    def spin(self, degrees, time):
        msg = OverrideRCIn()
        for i in range(time):
            msg.channels = [1500, 1500, 1500, 1900, 1500, 1500, 1500, 1500]  # Adjust for actual spin channels
        self.publish_and_log(msg, f"Spinning {degrees} degrees")
    
    def lightson(self):
        msg = OverrideRCIn()
        msg.channels = [1500] * 18
        msg.channels[8] = 2000
        msg.channels[9] = 2000          
        self.publish_and_log(msg, f"Lights On")

    def lightsoff(self):
        msg = OverrideRCIn()
        msg.channels = [1500] * 18
        msg.channels[8] = 1000
        msg.channels[9] = 1000          
        self.publish_and_log(msg, f"Lights Off")

    def flip(self, degrees, time):
        msg = OverrideRCIn()
        # Apparently channels 5,3 so literlly turn left and right at same time one - and one +
        for i in range(time):
            msg.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]  # Adjust for actual flip channels
        self.publish_and_log(msg, f"Flipping {degrees} degrees")

    def publish_and_log(self, msg, action):
        self.command_pub.publish(msg)
        self.get_logger().info(f"{action} - {msg.channels}")

    def execute_song(self):
        # Intro
        self.move_forward(5)
        self.wait(9)
        self.lightson()

        # Main
        self.move_left(5)
        self.wait(8)
        self.move_right(5)
        self.wait(4)
        self.spin(180, 5)
        self.wait(3)
        self.move_left(5)
        self.wait(3)
        self.move_right(5)
        self.wait(4)
        self.spin(180, 5)
        self.wait(3)
        self.move_left(5)
        self.wait(2)
        self.move_right(5)
        self.wait(2)
        self.down(5)
        self.wait(2)
        self.move_right(5)
        self.wait(2)
        self.up(5)
        self.wait(2)
        self.move_left(5)
        self.wait(2)
        self.flip(360, 5)
        self.wait(2)
        self.lightsoff()

        # Chorus
        for _ in range(4):
            self.move_left(5)
            self.wait(1)
            self.move_right(5)
            self.wait(1)
        for _ in range(4):
            self.move_right(5)
            self.wait(1)
            self.move_left(5)
            self.wait(1)

    def wait(self, seconds):
        self._set_neutral_all_channels()
        sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    dance_node = DanceNode()

    try:
        dance_node.execute_song()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        dance_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()