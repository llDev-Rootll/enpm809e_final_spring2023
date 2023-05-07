import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ariac_msgs.msg import CompetitionState
from competitor_interfaces.msg import Robots as RobotsMsg
from competitor_interfaces.srv import (
    EnterToolChanger
)
import time
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class StartCompetition(Node):
    '''
    Class for a robot commander node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        # Flag to indicate if the kit has been completed

        self._competition_started = False
        self._competition_state = None
        self._order_node_state = False

        # subscriber
        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)

        # timer
        self._robot_action_timer = self.create_timer(1, self._robot_action_timer_callback,
                                                     callback_group=timer_group)

        # Service client for starting the competition
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')        
        self.subscription = self.create_subscription(String,'order_node/status', self.listener_callback, 10)

    def listener_callback(self, msg):
        if msg.data == "READY":
            self._order_node_state = True

    def _competition_state_cb(self, msg: CompetitionState):
        '''
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        '''
        self._competition_state = msg.competition_state

    def _robot_action_timer_callback(self):
        '''
        Callback for the timer that triggers the robot actions
        '''
        if self._order_node_state:
            self.get_logger().info("Waiting for 3 seconds before starting competition")
            time.sleep(3)
            if self._competition_state == CompetitionState.READY and not self._competition_started:
                self.start_competition()

            if self._competition_started:
                self.get_logger().info("Destroying start node")
                self.destroy_node()
                return


    def start_competition(self):
        '''
        Start the competition
        '''
        self.get_logger().info('Waiting for competition state READY')

        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
            self._competition_started = True
        else:
            self.get_logger().warn('Unable to start competition')

def main(args=None):
    '''
    Main function for the floor robot.
    '''
    rclpy.init(args=args)
    node = StartCompetition()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()