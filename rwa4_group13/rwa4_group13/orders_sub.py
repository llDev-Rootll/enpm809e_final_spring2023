import rclpy
from rclpy.node import Node

from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from .datastructures import AriacOrder, PartPoses, TrayPoses


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

        self.timed_function = self.create_timer(0.1, self.robot_kitting)

        # Initiate the poses as None
        self.tray1_poses = None
        self.tray2_poses = None
        self.part1_poses = None
        self.part2_poses = None
        self.order = []


        self.order_id_to_pick = None
        self._kit_completed = False

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
            self.order_id_to_pick = "2"
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
                            sensor_pose = msg.sensor_pose)
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
                            sensor_pose = msg.sensor_pose)
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
                            sensor_pose = msg.sensor_pose)
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
                            sensor_pose = msg.sensor_pose)
            if self.part2_poses and self.part2_poses.poses and self.part2_poses.parts and self.part2_poses.sensor_pose:
                self.right_bin_msg = False


    def robot_kitting(self):
        """
        Print all the required information by parsing the objects created.
        Check if all the message flags are disabled and parse flag is enabled
        """

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
            tray_id = order_picked.kitting_task.tray_id
            cur_tray_pose = None

            for idx, t_id in enumerate(all_tray_ids):
                if t_id == tray_id:
                    cur_tray_pose = all_tray_poses[idx]
                    break

            assert cur_tray_pose is not None, "No trays matched tray ID in Order"
            
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
            output_part = {}

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
                            output_part["{} {}".format(self.color_dict[cur_part.color], self.type_dict[cur_part.type])] = cur_part_pose
                            output_part_string.append(output_part_n)
                            break
                    
            part_string = "".join("{}".format(part) for part in output_part_string)
            output = "{} \nTray:{} \nPart:\n{}".format(output_order, output_tray, part_string)
            # self.get_logger().info('%s' % output)
            # print(output_part)
            print("HERERE")
            
            self.parse_flag = False

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()