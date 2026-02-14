#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState
from typing import List, Optional


class LowStateFootArrayPrinter(Node):
    def __init__(self) -> None:
        super().__init__('lowstate_foot_array_printer', parameter_overrides=[])
        self.create_subscription(LowState, '/lowstate', self._callback, 10)
        self.get_logger().info('Printing /lowstate foot_force arrays only')

    def _callback(self, msg: LowState) -> None:
        # Convert fixed-size arrays to regular Python lists for clean terminal output.
        foot_force = [int(msg.foot_force[i]) for i in range(4)]
        foot_force_est = [int(msg.foot_force_est[i]) for i in range(4)]
        print(f'foot_force: {foot_force} | foot_force_est: {foot_force_est}', flush=True)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LowStateFootArrayPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
