import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from competitor_interfaces.msg import Robots as RobotsMsg
from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ariac_msgs.srv import (MoveAGV, ChangeGripper, VacuumGripperControl)
from competitor_interfaces.srv import (
    EnterToolChanger, ExitToolChanger, PickupTray, 
    MoveTrayToAGV, PlaceTrayOnAGV, RetractFromAGV,
    PickupPart, MovePartToAGV, PlacePartInTray
)

from .data_structures import AriacOrder, PartPoses, TrayPoses, OrderActionParams


class ARIACKitting(Node):
    """
    Class to instantiate rwa4 methods and attributes

    Args:
        Node (Class): Node creation

    Attributes
    ----------
    _color_dict (dict): Mapping the part color id to color name
    _type_dict (dict): Mapping the part type id to type name
    __table1_msg (bool): Flag to read the first message from topic /ariac/sensors/table1_camera/image 
    __table2_msg (bool): Flag to read the first message from topic /ariac/sensors/table2_camera/image 
    __left_bin_msg (bool): Flag to read the first message from topic /ariac/sensors/left_bins_camera/image 
    __right_bin_msg (bool): Flag to read the first message from topic /ariac/sensors/right_bins_camera/image
    __parse_flag (bool): Flag to enable printing information on terminal by parsing the objects

    Methods
    -------
    __init__():
        Constructor to initialize all the attributes
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
    parse_order():
        Print all the required information by parsing the objects created.
        Check if all the message flags are disabled and parse flag is enabled.
        Execute robot motion using service clients and the picked order information
    change_robot_gripper(gripper_type):
        Change robot gripper to part/tray type
    goto_tool_changer(robot, station, gripper_type):
        Move the end effector inside the gripper slot.
    lock_agv():
        Lock AGV and prepare to move it
    move_agv_to_warehouse(location):
        Move AGV to warehouse
    move_part_to_agv(robot, part_pose, agv, quadrant):
        Move part to AGV
    move_robot_home(robot_name):
        Move one of the robots to its home position.
        move_tray_to_agv(robot, tray_pose, agv)
    Move a tray to an AGV
    pickup_part(robot, part pose, part_type, part_color, bin_side)
        Pick up part from a bin
    pickup_tray(robot, tray_id, tray_pose, tray_table)
        Pick up a tray from a kitting station
    place_part_in_tray(robot, agv, quadrant)
        Place part in tray
    place_tray(robot, tray_id, agv)
        Place tray on AGV
    ready_callback()
        Timer callback for publishing on topic order_node/status
    retract_from_agv(robot, agv)
        Retract robot from AGV
    retract_from_tool_changer(robot, station, gripper_type)
        Retract robot gripper from the gripper slot
    robot_kitting(final_order_action)
        Call the services to perform kitting
    set_robot_gripper_state(state)
        Enable/Disable gripper suction
    """

    _color_dict = {0:"Red", 1:"Green", 2 : "Blue", 3: "Orange", 4: "Purple"}
    _type_dict = {10:"Battery", 11: "Pump", 12:"Sensor", 13:"Regulator"}

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
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        self.__kit_completed = False
        self._get_order_id = str

        ####################### Publisher for synchronizing with the start competition node ###########
        self._order_status_pub = self.create_publisher(String, 'order_node/status', 10)
        self._node_ready = self.create_timer(0.5, self.ready_callback)
        
        ######################### Parse Order ID required #####################################
        self.declare_parameter('order_id', rclpy.Parameter.Type.STRING)
        self._get_order_id = self.get_parameter('order_id').get_parameter_value().string_value
        # self.get_logger().info('Get order: %s' % self._get_order_id)

        ############################ Initialize Topic Subscribers ##################################
        self.__qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self._orders_sub = self.create_subscription(
            Order, '/ariac/orders', self.orders_callback, 10)
        
        self._table1_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table1_camera/image',
            self.table1_callback, self.__qos_policy)
        
        self._table2_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table2_camera/image',
            self.table2_callback, self.__qos_policy)
        
        self._left_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',
            self.left_bin_callback, self.__qos_policy)
        
        self._right_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',
            self.right_bin_callback, self.__qos_policy)
        
        ############################ Initiate the poses ##################################
        self.__tray1_poses = TrayPoses
        self.__tray2_poses = TrayPoses
        self.__part1_poses = PartPoses
        self.__part2_poses = PartPoses
        self.__order = []
        self.__order_id_to_pick = str

        self.__table1_msg = True
        self.__table2_msg = True
        self.__left_bin_msg = True
        self.__right_bin_msg = True
        self.__parse_flag = True

        ########### ######## Initialize Timer Callback for the Order ######################
        self._timed_function = self.create_timer(0.5, self.parse_order, callback_group=self._timer_group)

        ############################ Initialize Service Clients ##########################

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home_client = self.create_client(
            Trigger, '/competitor/floor_robot/go_home',
            callback_group=self._service_group)
        
        # Service client for entering the gripper slot
        self._goto_tool_changer_client = self.create_client(
            EnterToolChanger, '/competitor/floor_robot/enter_tool_changer',
            callback_group=self._service_group)
        
        # Service client for retracting from the gripper slot
        self._retract_from_tool_changer_client = self.create_client(
            ExitToolChanger, '/competitor/floor_robot/exit_tool_changer',
            callback_group=self._service_group)
        
        # Service client for retracting from agv
        self._retract_from_agv_client = self.create_client(
            RetractFromAGV, '/competitor/floor_robot/retract_from_agv',
            callback_group=self._service_group)

        # Service client for picking tray
        self._pickup_tray_client = self.create_client(
            PickupTray, '/competitor/floor_robot/pickup_tray',
            callback_group=self._service_group)
        
        # Service client for moving tray
        self._move_tray_to_agv_client = self.create_client(
            MoveTrayToAGV, '/competitor/floor_robot/move_tray_to_agv',
            callback_group=self._service_group)
        
        # Service client for placing tray
        self._place_tray_client = self.create_client(
            PlaceTrayOnAGV, '/competitor/floor_robot/place_tray_on_agv',
            callback_group=self._service_group)
        
        # Service client for picking tray
        self._pickup_part_client = self.create_client(
            PickupPart, '/competitor/floor_robot/pickup_part',
            callback_group=self._service_group)
        
        # Service client for moving part
        self._move_part_to_agv_client = self.create_client(
            MovePartToAGV, '/competitor/floor_robot/move_part_to_agv',
            callback_group=self._service_group)
        
        # Service client for placing part
        self._place_part_in_agv_client = self.create_client(
            PlacePartInTray, '/competitor/floor_robot/place_part_in_tray',
            callback_group=self._service_group)
        
        # Service client for changing gripper
        self._change_robot_gripper_client = self.create_client(
            ChangeGripper, '/ariac/floor_robot_change_gripper',
            callback_group=self._service_group)
        
        # Service client for enabling gripper
        self._enable_robot_gripper_client = self.create_client(
            VacuumGripperControl, '/ariac/floor_robot_enable_gripper',
            callback_group=self._service_group)
        

    #######################################################################
    ######################### To call Service client ######################
    #######################################################################
    def move_robot_home(self, robot_name:str) -> None: 
        '''
        Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''

        self.get_logger().info('Move robot to home service called')

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

        self.get_logger().info('Move inside gripper slot service called for {}'.format(gripper_type))

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
                self.get_logger().info('Robot is at the tool changer for {}'.format(gripper_type))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move the robot to the tool changer')

    def retract_from_tool_changer(self, robot, station, gripper_type):
        """ Retract robot gripper from the gripper slot

        Args:
            robot (str): Robot name
            station (str): Gripper station name
            gripper_type (str): Gripper type

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Retract robot gripper service called for {}'.format(gripper_type))

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
                self.get_logger().info('Robot is retracting from the tool changer for {}'.format(gripper_type))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract the robot from the tool changer')

    def change_robot_gripper(self, gripper_type:str):
        """
        Change robot gripper

        Args:
            gripper_type (str): Gripper type

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Change robot gripper service called for {}'.format(gripper_type))

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
        """
        Enable/Disable gripper suction

        Args:
            state (bool): Flag to Enable/Disable gripper suction

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Set robot gripper status service called')

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


    def pickup_tray(self, robot, tray_id, tray_pose, tray_table):
        """
        Pick up a tray from a kitting station

        Args:
            robot (str): Robot name
            tray_id (int): ID of the tray to be picked up
            tray_pose (Pose): pose of the tray in the world frame
            tray_table (str): Tray station: kts1 or kts2

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

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
                self.get_logger().info('Robot is picking up tray {} from table {}'.format(tray_id, tray_table))
                self.set_robot_gripper_state(True)

        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick tray {} by the robot'.format(tray_id))

    def move_tray_to_agv(self, robot, tray_pose, agv):
        """
        Move a tray to an AGV

        Args:
            robot (str): Robot name
            tray_pose (Pose): Pose of the tray in the world frame
            agv (str): AGV number

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """
        
        self.get_logger().info('Move tray to {} service called'.format(agv))

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
                self.get_logger().info('Robot is moving tray to {}'.format(agv))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move tray by the robot')

    def place_tray(self, robot, tray_id, agv):
        """
        Place tray on AGV

        Args:
            robot (str): Robot name: floor_robot
            tray_id (int): ID of the tray to be placed up
            agv (str): AGV number

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Place tray {} on {} service called'.format(tray_id, agv))

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
                self.get_logger().info('Robot is placing the tray {} on {}'.format(tray_id, agv))
                self.set_robot_gripper_state(False)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place the tray by the robot')

    def retract_from_agv(self, robot, agv):
        """
        Retract robot from AGV

        Args:
            robot (str): Robot name: floor_robot
            agv (str): AGV number

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Retract from {} service called'.format(agv))

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
                self.get_logger().info('Retracting from {}'.format(agv))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to retract from AGV')
          
    def pickup_part(self, robot, part_pose, part_type, part_color, bin_side):
        """ 
        Pick up part

        Args:
            robot (str): Robot name
            part_pose (Pose): Pose of the part in the world frame
            part_type (int): The type of the part
            part_color (int): The color of the part
            bin_side (str): Left/Right corresponds to which camera sees the part

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Pick up part service called')

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
                self.get_logger().info('Robot is picking part from {}'.format(bin_side))
                self.set_robot_gripper_state(True)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to pick part')     

    def move_part_to_agv(self, robot, part_pose, agv, quadrant):
        """
        Move part to AGV

        Args:
            robot (str): Robot name
            part_pose (Pose): Pose of the part in the world frame
            agv (str): AGV number
            quadrant (int): The location of the part within the tray

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Move part to {} service called'.format(agv))

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
                self.get_logger().info('Robot is moving part to {}'.format(agv))
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to move part to {}'.format(agv))          

    def place_part_in_tray(self, robot, agv, quadrant):
        """
        Place part in tray

        Args:
            robot (str): Robot name
            agv (str): AGV number
            quadrant (int): The location of the part within the tray

        Raises:
            ValueError: Error raised if robot is invalid
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

        self.get_logger().info('Place part in tray service called')

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
                self.get_logger().info('Robot is placing part on quadrant {} of tray on {}'.format(quadrant, agv))
                self.set_robot_gripper_state(False)
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
            self.get_logger().error('Unable to place part in tray')  

    def lock_agv(self):
        """
        Lock AGV

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

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
        """
        Move AGV to warehouse

        Args:
            location (int): The location to send the AGV when the kit is complete.

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        """

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

    ##########################################################
    ######################### Callbacks ######################
    ##########################################################

    def ready_callback(self):
        """
        Timer callback for publishing on topic order_node/status
        """

        msg = String()
        msg.data = 'READY' 
        self._order_status_pub.publish(msg)

    def orders_callback(self, msg):
        """
        Create AriacOrder object and enable message flags to read messgaes from other topics

        Args:
            msg (Order): Order read from topic /ariac/orders
        """
        
        self.__order.append(AriacOrder(order_id = msg.id,
                      order_type = msg.type,
                      order_priority = msg.priority,
                      kitting_task = msg.kitting_task
                      ))
        
        if len(self.__order) == 4:
            self.__order_id_to_pick = self._get_order_id

            # enable the flags to log only the first message
            self.__table1_msg = True
            self.__table2_msg = True
            self.__left_bin_msg = True
            self.__right_bin_msg = True
            self.__parse_flag = True


    def table1_callback(self, msg):
        """
        Create TrayPoses object, check if the message is valid and deactivate table1_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Tray information from camera 1
        """

        if self.__table1_msg:
            self.__tray1_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose,
                            tray_table = "kts1")
            if self.__tray1_poses and self.__tray1_poses._poses and self.__tray1_poses._sensor_pose and self.__tray1_poses._ids:
                self.__table1_msg = False

    def table2_callback(self, msg):
        """
        Create TrayPoses object, check if the message is valid and deactivate table2_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Tray information from camera 2
        """

        if self.__table2_msg:
            self.__tray2_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose,
                            tray_table = "kts2")
            if self.__tray2_poses and self.__tray2_poses._poses and self.__tray2_poses._sensor_pose and self.__tray2_poses._ids:
                self.__table2_msg = False
                

    def left_bin_callback(self, msg):
        """
        Create PartPoses object, check if the message is valid and deactivate part1_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Parts information from left bin
        """

        if self.__left_bin_msg:
            self.__part1_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose,
                            part_bin = "left_bins")
            if self.__part1_poses and self.__part1_poses._poses and self.__part1_poses._parts and self.__part1_poses._sensor_pose:
                self.__left_bin_msg = False

    def right_bin_callback(self, msg):
        """
        Create PartPoses object, check if the message is valid and deactivate part2_msg flag

        Args:
            msg (AdvancedLogicalCameraImage): Parts information from right bin
        """

        if self.__right_bin_msg:
            self.__part2_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose,
                            part_bin = "right_bins")
            if self.__part2_poses and self.__part2_poses._poses and self.__part2_poses._parts and self.__part2_poses._sensor_pose:
                self.__right_bin_msg = False

    def parse_order(self):
        """
        Print all the required information by parsing the objects created.
        Check if all the message flags are disabled and parse flag is enabled.
        Execute robot motion using service clients and the picked order information
        """

        if self.__kit_completed:
            return
        
        if ((not self.__table1_msg) and (not self.__table2_msg) and (not self.__left_bin_msg) and (not self.__right_bin_msg)) and self.__parse_flag:
            
            ############ Order ##########
            order_picked = [order for order in self.__order 
                            if order._id == self.__order_id_to_pick][0]

            output_order = "\n\n----------------------\n--- Order {} ---\n----------------------\n".format(order_picked._id)

            ############ Tray ##########
            all_tray_ids = [*self.__tray1_poses._ids, *self.__tray2_poses._ids]
            
            all_tray_poses = [*self.__tray1_poses._poses, *self.__tray2_poses._poses]
            all_tray_tables = [*self.__tray1_poses._tray_tables, *self.__tray2_poses._tray_tables]

            tray_id = order_picked._kitting_task._tray_id
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
                                                   agv_number="agv{}".format(str(order_picked._kitting_task._agv_number)),
                                                   destination=order_picked._kitting_task._destination
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
            all_part_parts = [*self.__part1_poses._parts, *self.__part2_poses._parts]
            all_part_poses = [*self.__part1_poses._poses, *self.__part2_poses._poses]
            output_part_string = []
            part_history = []
            for cur_part in order_picked._kitting_task._parts:
                for part_idx, part_parts in enumerate(all_part_parts):
                    if cur_part._color == part_parts._color and cur_part._type == part_parts._type:
                            cur_part_pose = all_part_poses[part_idx]
                            if cur_part_pose in part_history:
                                continue
                            else:
                                part_history.append(cur_part_pose)
                            
                            output_part_n = "  - {} {}\n    - pose:\n      - position: [{}, {}, {}] \n      - orientation: [{}, {}, {}, {}]\n".format(self._color_dict[cur_part._color], 
                                                                                                                    self._type_dict[cur_part._type],
                                                                                                                    cur_part_pose.position.x, 
                                                                                                                    cur_part_pose.position.y, 
                                                                                                                    cur_part_pose.position.z, 
                                                                                                                    cur_part_pose.orientation.x, 
                                                                                                                    cur_part_pose.orientation.y, 
                                                                                                                    cur_part_pose.orientation.z,
                                                                                                                    cur_tray_pose.orientation.w)
                            
                            final_order_action.add_parts(part_type=cur_part._type,
                                                         part_color=cur_part._color,
                                                         part_quadrant=cur_part._quadrant,
                                                         part_bin=part_parts._part_bin,
                                                         part_pose=cur_part_pose
                            )
                            output_part_string.append(output_part_n)
                            break
                    
            part_string = "".join("{}".format(part) for part in output_part_string)
            output = "{} \nTray:{} \nPart:\n{}".format(output_order, output_tray, part_string)
            self.get_logger().info("Order {} picked and parsed".format(order_picked._id))
            self.get_logger().info('%s' % output)
            self.__parse_flag = False

            # Create AGV specific service client clients
            self._lock_agv_client = self.create_client(
                        Trigger, '/ariac/{}_lock_tray'.format(final_order_action._agv_number),
                        callback_group=self._service_group)

            self._move_agv_to_warehouse_client = self.create_client(
                MoveAGV, '/ariac/move_{}'.format(final_order_action._agv_number),
                callback_group=self._service_group)

            # Start Robot kitting after all the order information is received
            self.robot_kitting(final_order_action=final_order_action)
            
            self.__kit_completed = True

            
    def robot_kitting(self, final_order_action):
        """
        Call the services to perform kitting

        Args:
            final_order_action (OrderActionParams): Object consisting all the order infromation
        """

        # move robot home
        self.move_robot_home("floor_robot")
        
        # change gripper type
        self.goto_tool_changer("floor_robot", final_order_action._tray_table, "trays")
        self.change_robot_gripper("tray")
        self.retract_from_tool_changer("floor_robot", final_order_action._tray_table, "trays")

        # pick and place tray
        self.pickup_tray("floor_robot", final_order_action._tray_id, final_order_action._tray_pose, final_order_action._tray_table)
        self.move_tray_to_agv("floor_robot", final_order_action._tray_pose, final_order_action._agv_number)
        self.place_tray("floor_robot", final_order_action._tray_id, final_order_action._agv_number)
        self.retract_from_agv("floor_robot", final_order_action._agv_number)

        # change gripper to pick up parts
        self.goto_tool_changer("floor_robot", final_order_action._tray_table, "parts")
        self.change_robot_gripper("part")
        self.retract_from_tool_changer("floor_robot", final_order_action._tray_table, "parts")

        # for loop to go through all the parts
        for cur_part in final_order_action._parts:

            # pick and place purple pump
            self.pickup_part("floor_robot", cur_part[4], cur_part[0], cur_part[1], cur_part[3])
            self.move_part_to_agv("floor_robot", cur_part[4], final_order_action._agv_number, cur_part[2])
            self.place_part_in_tray("floor_robot", final_order_action._agv_number, cur_part[2])
            self.retract_from_agv("floor_robot", final_order_action._agv_number)

            # move robot home
            self.move_robot_home("floor_robot")

        # move agv to warehouse
        self.lock_agv()
        self.move_agv_to_warehouse(final_order_action._destination)

def main(args=None):
    rclpy.init(args=args)
    my_node = ARIACKitting()
    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        pass
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()