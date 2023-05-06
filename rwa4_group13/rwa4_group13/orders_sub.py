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
from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from .datastructures import AriacOrder, PartPoses, TrayPoses, OrderActionParams


class rwa4(Node):
    """
    Class to instantiate rwa4 methods and attributes

    Args:
        Node (Class): Node creation

    Attributes
    ----------
    color_dict (dict): Mapping the part color id to color name
    type_dict (dict): Mapping the part type id to type name
    table1_msg (bool): Flag to read the first message from topic /ariac/sensors/table1_camera/image 
    table2_msg (bool): Flag to read the first message from topic /ariac/sensors/table2_camera/image 
    left_bin_msg (bool): Flag to read the first message from topic /ariac/sensors/left_bins_camera/image 
    right_bin_msg (bool): Flag to read the first message from topic /ariac/sensors/right_bins_camera/image
    parse_flag (bool): Flag to enable printing information on terminal by parsing the objects

    Methods
    -------
    orders_callback(msg): 
        Create AriacOrder object and save the attributes from msg parameter
    table1_callback(msg):
        Create TrayPoses object and save the attributes from msg parameter
    table2_callback(msg):
        Create TrayPoses object and save the attributes from msg parameter
    left_bin_callback(msg):
        Create PartPoses object and save the attributes from msg parameter
    right_bin_callback(msg):
        Create PartPoses object and save the attributes from msg parameter
    parse_print():
        Print all the required information by parsing the objects created
    """

    color_dict = {0:"Red", 1:"Green", 2 : "Blue", 3: "Orange", 4: "Purple"}
    type_dict = {10:"Battery", 11: "Pump", 12:"Sensor", 13:"Regulator"}
    table1_msg = False
    table2_msg = False
    left_bin_msg = False
    right_bin_msg = False
    parse_flag = False
    order_picked = False

    def __init__(self):
        """
        Constructs all the necessary attributes for the rwa4 node
        Creates subscribers to the required topics
        """

        super().__init__('rwa4')
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time])
        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        self._kit_completed = False
        self._competition_started = False
        self._competition_state = None

        self.create_subscription(CompetitionState, '/ariac/competition_state',
                                 self._competition_state_cb, 1)
        

        #############################################################
        # Service client for starting the competition
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',
            callback_group=service_group)
        
        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=service_group)
        
        self._retract_from_tool_changer_client
        self._retract_from_agv_client

        self._pickup_tray_client
        self._move_tray_to_agv_client
        self._place_tray_client
        

        self._pickup_part_client
        self._move_part_to_agv_client
        self._plave_part_in_agv_client

        self._lock_agv_client
        self._move_agv_to_warehouse_client


        ##############################################################
        self.declare_parameter('order_id', rclpy.Parameter.Type.STRING)
        self._get_order_id = self.get_parameter('order_id').get_parameter_value().string_value
        
        self.get_logger().info('Get order: %s' % self._get_order_id)

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.orders_sub = self.create_subscription(
            Order, '/ariac/orders', self.orders_callback, 10)
        
        self.table1_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table1_camera/image',
            self.table1_callback, qos_policy)
        
        self.table2_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table2_camera/image',
            self.table2_callback, qos_policy)
        
        self.left_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',
            self.left_bin_callback, qos_policy)
        
        self.right_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',
            self.right_bin_callback, qos_policy)

        self.timed_function = self.create_timer(1, self.parse_order,callback_group=timer_group)

        # Initiate the poses as None
        self.tray1_poses = None
        self.tray2_poses = None
        self.part1_poses = None
        self.part2_poses = None
        self.order = []


        self.order_id_to_pick = None
    
    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self._move_floor_robot_home_client.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)

    def _competition_state_cb(self, msg: CompetitionState):
        '''
        /ariac/competition_state topic callback function

        Args:
            msg (CompetitionState): CompetitionState message
        '''
        self._competition_state = msg.competition_state

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
    
    def orders_callback(self, msg):
        """
        Create AriacOrder object and enable message flags to read messgaes from other topics

        Args:
            msg (Order): Order read from topic /ariac/orders
        """
        
        self.order.append(AriacOrder(order_id = msg.id,
                      order_type = msg.type,
                      order_priority = msg.priority,
                      kitting_task = msg.kitting_task
                      ))

        if len(self.order) == 4:
            self.order_id_to_pick = self._get_order_id
            # enable the flags to log only the first message
            self.table1_msg = True
            self.table2_msg = True
            self.left_bin_msg = True
            self.right_bin_msg = True
            self.parse_flag = True


    def table1_callback(self, msg):
        """
        Create TrayPoses object, check if the message is valid and deactivate table1_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Tray information from camera 1
        """

        if self.table1_msg:
            self.tray1_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose,
                            tray_table = "kts1")
            if self.tray1_poses and self.tray1_poses.poses and self.tray1_poses.sensor_pose and self.tray1_poses.ids:
                self.table1_msg = False

    def table2_callback(self, msg):
        """
        Create TrayPoses object, check if the message is valid and deactivate table2_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Tray information from camera 2
        """

        if self.table2_msg:
            self.tray2_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose,
                            tray_table = "kts2")
            if self.tray2_poses and self.tray2_poses.poses and self.tray2_poses.sensor_pose and self.tray2_poses.ids:
                self.table2_msg = False
                

    def left_bin_callback(self, msg):
        """
        Create PartPoses object, check if the message is valid and deactivate part1_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Parts information from left bin
        """

        if self.left_bin_msg:
            self.part1_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose,
                            part_bin = "left_bins")
            if self.part1_poses and self.part1_poses.poses and self.part1_poses.parts and self.part1_poses.sensor_pose:
                self.left_bin_msg = False

    def right_bin_callback(self, msg):
        """
        Create PartPoses object, check if the message is valid and deactivate part2_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Parts information from right bin
        """

        if self.right_bin_msg:
            self.part2_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose,
                            part_bin = "right_bins")
            if self.part2_poses and self.part2_poses.poses and self.part2_poses.parts and self.part2_poses.sensor_pose:
                self.right_bin_msg = False


    def parse_order(self):
        """
        Print all the required information by parsing the objects created.
        Check if all the message flags are disabled and parse flag is enabled
        """
        if self._competition_state == CompetitionState.READY and not self._competition_started:
            self.start_competition()

        if self._kit_completed:
            return 
        
        if ((not self.table1_msg) and (not self.table2_msg) and (not self.left_bin_msg) and (not self.right_bin_msg)) and self.parse_flag:
            ############ Order ##########

            order_picked = [order for order in self.order 
                            if order.id == self.order_id_to_pick][0]

            output_order = "\n\n----------------------\n--- Order {} ---\n----------------------\n".format(order_picked.id)

            ############ Tray ##########
            all_tray_ids = [*self.tray1_poses.ids, *self.tray2_poses.ids]
            
            all_tray_poses = [*self.tray1_poses.poses, *self.tray2_poses.poses]
            all_tray_tables = [*self.tray1_poses.tray_tables, *self.tray2_poses.tray_tables]

            tray_id = order_picked.kitting_task.tray_id
            cur_tray_pose = None
            cur_tray_table = None

            for idx, t_id in enumerate(all_tray_ids):
                if t_id == tray_id:

                    cur_tray_table = all_tray_tables[idx]
                    cur_tray_pose = all_tray_poses[idx]
                    
                    break
            
            assert cur_tray_pose is not None, "No trays matched tray ID in Order"
            final_order_action = OrderActionParams(tray_id=tray_id,
                                                   tray_table=cur_tray_table,
                                                   agv_number="agv{}".format(str(order_picked.kitting_task.agv_number)),
                                                   )
            output_tray = "\n  - id: {} \n  - pose:\n    - position: [{}, {}, {}] \n    - orientation: [{}, {}, {}, {}]".format(tray_id, 
                                                                                                                  cur_tray_pose.position.x, 
                                                                                                                cur_tray_pose.position.y, 
                                                                                                                cur_tray_pose.position.z, 
                                                                                                                cur_tray_pose.orientation.x, 
                                                                                                                cur_tray_pose.orientation.y, 
                                                                                                                cur_tray_pose.orientation.z,
                                                                                                                cur_tray_pose.orientation.w)
            ############ Parts ############
            all_part_parts = [*self.part1_poses.parts, *self.part2_poses.parts]
            all_part_poses = [*self.part1_poses.poses, *self.part2_poses.poses]
            output_part_string = []

            for cur_part in order_picked.kitting_task.parts:
                for part_idx, part_parts in enumerate(all_part_parts):
                    if cur_part.color == part_parts.color and cur_part.type == part_parts.type:
                            cur_part_pose = all_part_poses[part_idx]

                            output_part_n = "  - {} {}\n    - pose:\n      - position: [{}, {}, {}] \n      - orientation: [{}, {}, {}, {}]\n".format(self.color_dict[cur_part.color], 
                                                                                                                    self.type_dict[cur_part.type],
                                                                                                                    cur_part_pose.position.x, 
                                                                                                                    cur_part_pose.position.y, 
                                                                                                                    cur_part_pose.position.z, 
                                                                                                                    cur_part_pose.orientation.x, 
                                                                                                                    cur_part_pose.orientation.y, 
                                                                                                                    cur_part_pose.orientation.z,
                                                                                                                    cur_tray_pose.orientation.w)
                            final_order_action.add_parts(part_type=cur_part.type,
                                                         part_color=cur_part.color,
                                                         part_quadrant=cur_part.part_bin,
                                                         part_pose=cur_part_pose
                            )
                            output_part_string.append(output_part_n)
                            break
                    
            part_string = "".join("{}".format(part) for part in output_part_string)
            output = "{} \nTray:{} \nPart:\n{}".format(output_order, output_tray, part_string)
            self.get_logger().info("Order {} picked and parsed".format(order_picked.id))
            self.get_logger().info('Order %s' % output)
            self.parse_flag = False

            self.robot_kitting(final_order_action=final_order_action)

            self._kit_completed = True

            
    def robot_kitting(self, final_order_action):

        # move robot home
        self.move_robot_home("floor_robot")
        self._goto_tool_changer_client("floor_robot", final_order_action.tray_table, "trays")

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()