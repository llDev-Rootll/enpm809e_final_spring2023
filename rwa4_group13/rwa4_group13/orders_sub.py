import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from competitor_interfaces.msg import Robots as RobotsMsg
from ariac_msgs.msg import (Order, CompetitionState)
from ariac_msgs.msg import AdvancedLogicalCameraImage

from std_srvs.srv import Trigger
from ariac_msgs.srv import (MoveAGV, ChangeGripper, VacuumGripperControl)
from competitor_interfaces.srv import (
    EnterToolChanger, ExitToolChanger, PickupTray, 
    MoveTrayToAGV, PlaceTrayOnAGV, RetractFromAGV,
    PickupPart, MovePartToAGV, PlacePartInTray
)

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
        self.service_group = MutuallyExclusiveCallbackGroup()

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
            callback_group=self.service_group)
        
        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=self.service_group)
        
        self._retract_from_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',
            callback_group=self.service_group)
        
        self._retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv',
            callback_group=self.service_group)

        self._pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray',
            callback_group=self.service_group)
        
        self._move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv',
            callback_group=self.service_group)
        
        self._place_tray_client = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv',
            callback_group=self.service_group)
        
        self._pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part',
            callback_group=self.service_group)
        
        self._move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv',
            callback_group=self.service_group)
        
        self._place_part_in_agv_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray',
            callback_group=self.service_group)
        
        # self._lock_agv_client = self.create_client(
        #     Trigger, '/ariac/agv_lock_tray',
        #     callback_group=self.service_group)
        
        # self._move_agv_to_warehouse_client = self.create_client(
        #     MoveAGV, '/ariac/move_agvX',
        #     callback_group=self.service_group)
        
        self._change_robot_gripper_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper',
            callback_group=self.service_group)
        
        self._enable_robot_gripper_client = self.create_client(
            VacuumGripperControl, '/ariac/floor_robot_enable_gripper',
            callback_group=self.service_group)

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

        # self.timed_function = self.create_timer(1, self.parse_order,callback_group=timer_group)
        self.timed_function = self.create_timer(0.1, self.parse_order)

        # Initiate the poses as None
        self.tray1_poses = None
        self.tray2_poses = None
        self.part1_poses = None
        self.part2_poses = None
        self.order = []


        self.order_id_to_pick = None
    
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
    
    def change_robot_gripper(self, gripper_type):
        
        request = ChangeGripper.Request()
        if gripper_type == "part":
            request.gripper_type = ChangeGripper.Request.PART_GRIPPER
        else:
            request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER

        future = self._change_robot_gripper_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot gripper changed to {}'.format(gripper_type))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to change gripper')
    
    def set_robot_gripper_state(self, state):
        
        request = VacuumGripperControl.Request()

        request.enable = state

        future = self._enable_robot_gripper_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                if state:
                    self.get_logger().info('Robot gripper state enabled')
                else:
                    self.get_logger().info('Robot gripper state disabled')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to change gripper state')

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
   
    def goto_tool_changer(self, robot, station, gripper_type):
        '''
        Move the end effector inside the gripper slot.

        Args:
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''

        self.get_logger().info('Move inside gripper slot service called')

        request = EnterToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._goto_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is at the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')

    def retract_from_tool_changer(self, robot, station, gripper_type):

        self.get_logger().info('Retract robot gripper service called')

        request = ExitToolChanger.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.station = station
        request.gripper_type = gripper_type

        future = self._retract_from_tool_changer_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is retracting from the tool changer')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract the robot from the tool changer')

    def pickup_tray(self, robot, tray_id, tray_pose, tray_table):

        self.get_logger().info('Pick up tray service called')

        request = PickupTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.tray_pose = tray_pose
        request.tray_station = tray_table

        future = self._pickup_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is picking up tray {}'.format(tray_id))
                self.set_robot_gripper_state(True)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick tray {} by the robot'.format(tray_id))

    def move_tray_to_agv(self, robot, tray_pose, agv):
        self.get_logger().info('Move tray to AGV service called')

        request = MoveTrayToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_pose = tray_pose
        request.agv = agv

        future = self._move_tray_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is moving tray')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move tray by the robot')

    def place_tray(self, robot, tray_id, agv):
        self.get_logger().info('Place tray on AGV service called')

        request = PlaceTrayOnAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.tray_id = tray_id
        request.agv = agv

        future = self._place_tray_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is placing the tray')
                self.set_robot_gripper_state(False)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place the tray by the robot')

    def retract_from_agv(self, robot, agv):
        self.get_logger().info('Retract from AGV service called')

        request = RetractFromAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv

        future = self._retract_from_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Retracting from AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract from AGV')
          
    def pickup_part(self, robot, part_pose, part_type, part_color, bin_side):
        self.get_logger().info('pick up part service called')

        request = PickupPart.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_type = part_type
        request.part_pose = part_pose
        request.part_color = part_color
        request.bin_side = bin_side

        future = self._pickup_part_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is picking part')
                self.set_robot_gripper_state(True)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick part')     

    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        self.get_logger().info('move part to agv service called')

        request = MovePartToAGV.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.part_pose = part_pose
        request.agv = agv
        request.quadrant = quadrant

        future = self._move_part_to_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is moving part to agv')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move part to agv')          

    def place_part_in_tray(self, robot, agv, quadrant):
        self.get_logger().info('place part in tray service called')

        request = PlacePartInTray.Request()

        if robot == "floor_robot":
            request.robot = RobotsMsg.FLOOR_ROBOT
        else:
            raise ValueError('Invalid robot name')

        request.agv = agv
        request.quadrant = quadrant

        future = self._place_part_in_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Robot is placing part in tray')
                self.set_robot_gripper_state(False)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place part in tray')  

    def lock_agv(self):

        self.get_logger().info('Lock AGV service called')

        request = Trigger.Request()

        future = self._lock_agv_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Locking AGV')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to lock AGV')
        
    def move_agv_to_warehouse(self, location):

        self.get_logger().info('Move AGV to warehouse service called')

        request = MoveAGV.Request()

        request.location = location

        future = self._move_agv_to_warehouse_client.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info('Moving AGV to warehouse')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move AGV to warehouse')
  
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
        # if self._competition_state == CompetitionState.READY and not self._competition_started:
        #     self.start_competition()

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
                                                   tray_pose=cur_tray_pose,
                                                   agv_number="agv{}".format(str(order_picked.kitting_task.agv_number)),
                                                   destination=order_picked.kitting_task.destination
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
                                                         part_quadrant=cur_part.quadrant,
                                                         part_bin=part_parts.part_bin,
                                                         part_pose=cur_part_pose
                            )
                            output_part_string.append(output_part_n)
                            break
                    
            part_string = "".join("{}".format(part) for part in output_part_string)
            output = "{} \nTray:{} \nPart:\n{}".format(output_order, output_tray, part_string)
            self.get_logger().info("Order {} picked and parsed".format(order_picked.id))
            self.get_logger().info('Order %s' % output)
            self.parse_flag = False

            self._lock_agv_client = self.create_client(
                        Trigger, '/ariac/{}_lock_tray'.format(final_order_action.agv_number),
                        callback_group=self.service_group)

            self._move_agv_to_warehouse_client = self.create_client(
                MoveAGV, '/ariac/move_{}'.format(final_order_action.agv_number),
                callback_group=self.service_group)

            self.robot_kitting(final_order_action=final_order_action)
            
            self._kit_completed = True

            
    def robot_kitting(self, final_order_action):

        # move robot home
        self.move_robot_home("floor_robot")
        
        # change gripper type
        self.goto_tool_changer("floor_robot", final_order_action.tray_table, "trays")
        self.change_robot_gripper("tray")
        self.retract_from_tool_changer("floor_robot", final_order_action.tray_table, "trays")

        # pick and place tray
        self.pickup_tray("floor_robot", final_order_action.tray_id, final_order_action.tray_pose, final_order_action.tray_table)
        self.move_tray_to_agv("floor_robot", final_order_action.tray_pose, final_order_action.agv_number)
        self.place_tray("floor_robot", final_order_action.tray_id, final_order_action.agv_number)
        self.retract_from_agv("floor_robot", final_order_action.agv_number)

        # change gripper to pick up parts
        self.goto_tool_changer("floor_robot", final_order_action.tray_table, "parts")
        self.change_robot_gripper("part")
        self.retract_from_tool_changer("floor_robot", final_order_action.tray_table, "parts")

        # for loop to go through all the parts
        for cur_part in final_order_action.parts:
            
            # pick and place purple pump
            self.pickup_part("floor_robot", cur_part[4], cur_part[0], cur_part[1], "right_bins")
            self.move_part_to_agv("floor_robot", cur_part[4], final_order_action.agv_number, cur_part[2])
            self.place_part_in_tray("floor_robot", final_order_action.agv_number, cur_part[2])
            self.retract_from_agv("floor_robot", final_order_action.agv_number)

            # move robot home
            self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.lock_agv()
        self.move_agv_to_warehouse(final_order_action.destination)

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()