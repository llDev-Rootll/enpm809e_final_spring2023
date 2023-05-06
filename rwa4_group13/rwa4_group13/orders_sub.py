import rclpy
from rclpy.node import Node

from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
import PyKDL

class Part():
    """
    Class to instantiate Part attributes

    Methods
    -------
    __str__()
        Print the attributes of the Part class
    """

    def __init__(self, part_color, part_type, **kwargs) -> None:
        """
        Constructs all the necessary attributes for the Part object

        Args:
            part_color (int): The color of the part
            part_type (int): The type of the part
            quadrant (int): The location of the part within the tray
        """

        self.color = part_color
        self.type = part_type
        self.quadrant = kwargs["part_quadrant"] if kwargs else None
    
    def __str__(self) -> str:
        """
        Returns a string with the attributes of the Part class

        Returns:
            str: String with all the attributes information
        """

        if self.quadrant:
            return "color : {},\n\t type : {},\n\t quadrant : {}\n\t".format(self.color, self.type, self.quadrant)
        else:
            return "color : {},\n\t type : {}\n\t".format(self.color, self.type)

class Kitting_Task():
    """
    Class to instantiate Knitting_Task attributes

    Methods
    -------
    __str__()
        Print the attributes of the Knitting_Task class
    """

    def __init__(self, agv_number, tray_id, destination, parts) -> None:
        """
        Constructs all the necessary attributes for the Kitting_Task object

        Args:
            agv_number (int): The AGV to use to build the kit (1, 2, 3, or 4)
            tray_id (int): The tray to use to build the kit
            destination (int): The destination to send the AGV when the kit is complete.
            parts (list): Information on parts needed to build the kit
        """

        self.agv_number = agv_number
        self.tray_id = tray_id
        self.destination = destination
        self.parts = [Part(part_color = kitting_part.part.color, 
                           part_type = kitting_part.part.type, 
                           part_quadrant = kitting_part.quadrant) 
                           for kitting_part in parts]
    
    def __str__(self) -> str:
        """
        Returns a string with the attributes of the Knitting_Task class

        Returns:
            str: String with all the attributes information
        """

        part_string = "".join("part_{} : \n\t{}".format(str(idx), part.__str__()) for idx, part in enumerate(self.parts))
        output = "agv_number : {},\n tray_id : {},\n destination : {},\n parts : \n\t{}".format(self.agv_number, self.tray_id, self.destination, part_string)

        return output
    
class AriacOrder():
    """
    Class to instantiate AriacOrder attributes

    Methods
    -------
    __str__()
        Print the attributes of the AriacOrder class
    """

    def __init__(self, order_id, order_type, order_priority, kitting_task) -> None:
        """
        Constructs all the necessary attributes for the AriacOrder object

        Args:
            order_id (int): Unique order id
            order_type (int): Type of the task, Kitting: 0
            order_priority (int): The priority of this Order (high priority: 1 and low priority: 0). 
                                  If multiple Orders are announced, the ones with high priorities must be built ﬁrst.
            kitting_task (Kitting_Task): Information of the kit attributes
        """

        self.id = order_id 
        self.type = order_type
        self.priority = order_priority
        self.kitting_task = Kitting_Task(agv_number = kitting_task.agv_number,
                                         tray_id = kitting_task.tray_id,
                                         destination = kitting_task.destination,
                                         parts = kitting_task.parts
                                         )

    def __str__(self) -> str:
        """
        Returns a string with the attributes of the AriacOrder class

        Returns:
            str: String with all the attributes information
        """

        output = "id : {},\n type : {},\n priority : {},\n kitting_task : {}".format(self.id, self.type, self.priority, self.kitting_task.__str__())
        
        return output

class Position():
    """
    Class to instantiate Position attributes

    Methods
    -------
    __str__()
        Print the attributes of the Position class
    """

    def __init__(self, x, y, z) -> None:
        """
        Constructs all the necessary attributes for the Position object

        Args:
            x (float): Point Position in x axis
            y (float): Point Position in y axis
            z (float): Point Position in z axis
        """

        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self) -> str:
        """
        Returns a string with the attributes of the Position class

        Returns:
            str: String with all the attributes information
        """
        
        return "position: \n\t x : {},\n\t y : {},\n\t z : {}\n\t".format(self.x, self.y, self.z)

class Orientation():
    """
    Class to instantiate Orientation attributes

    Methods
    -------
    __str__()
        Print the attributes of the Orientation class
    """

    def __init__(self, x, y, z, w) -> None:
        """
        Constructs all the necessary attributes for the Orientation object

        Args:
            x (float): Quaternion orientation in x axis
            y (float): Quaternion orientation in y axis
            z (float): Quaternion orientation in z axis
            w (float): Quaternion orientation in w axis
        """

        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def __str__(self) -> str:
        """
        Returns a string with the attributes of the Orientation class

        Returns:
            str: String with all the attributes information
        """
        
        return "orientation: \n\t x : {},\n\t y : {},\n\t z : {},\n\t w : {}\n\t".format(self.x, self.y, self.z, self.w)

class ObjectPose():
    """
    Class to instantiate ObjectPose attributes

    Methods
    -------
    __str__()
        Print the attributes of the ObjectPose class
    """
    
    def __init__(self, object_position, object_orientation) -> None:
        """
        Constructs all the necessary attributes for the ObjectPose object

        Args:
            object_position (Point): position of a point in free space
            object_orientation (Quaternion): orientation in free space in quaternion form
        """

        self.position = Position(x = object_position.x,
                                 y = object_position.y,
                                 z = object_position.z)
        
        self.orientation = Orientation(x = object_orientation.x,
                                       y = object_orientation.y,
                                       z = object_orientation.z,
                                       w = object_orientation.w)
    
    def __str__(self) -> str:
        """
        Returns a string with the attributes of the ObjectPose class

        Returns:
            str: String with all the attributes information
        """

        return "{}\n{}".format(self.position.__str__(), self.orientation.__str__())

class WorldFrame():
    """
    Class to convert object position in camera frame to world frame

    Methods
    -------
    _multiply_pose(pose1, pose2):
        convert object position in camera frame to world frame
    """

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together. Function taken from tf_node.py
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose        

class TrayPoses(WorldFrame):
    """
    Class to instantiate TrayPoses attributes

    Args:
        WorldFrame (Class): Used to convert tray poses to be with respect to world frame
    """

    def __init__(self, tray_poses, sensor_pose) -> None:
        """
        Constructs all the necessary attributes for the TrayPoses object
        Converts tray poses to be with respect to world frame 

        Args:
            tray_poses (list): Information on trays that are detected by the camera
            sensor_pose (Pose): Pose of the camera in the world frame
        """
        
        self.ids = []
        self.poses = []
        for tray_pose in tray_poses:

            self.ids.append(tray_pose.id)
            tray_world_pose = self._multiply_pose(sensor_pose, tray_pose.pose)
            self.poses.append(ObjectPose(object_position = tray_world_pose.position,
                                        object_orientation = tray_world_pose.orientation))
        
        self.sensor_pose = ObjectPose(object_position = sensor_pose.position,
                                      object_orientation= sensor_pose.orientation)

    def __str__(self) -> str:
        """
        Returns a string with the attributes of the TrayPoses class

        Returns:
            str: String with all the attributes information
        """

        output = "tray_poses: \n"+"".join("tray_id : {}\n{}".format(str(tray_id), pose.__str__()) for tray_id, pose in zip(self.ids, self.poses)) 

        return output + "\n\t sensor pose: \n{}".format(self.sensor_pose.__str__())

class PartPoses(WorldFrame):
    """
    Class to instantiate PartPoses attributes

    Args:
        WorldFrame (Class): Used to convert part poses to be with respect to world frame
    """

    def __init__(self, part_poses, sensor_pose) -> None:
        """        
        Constructs all the necessary attributes for the PartPoses object
        Converts part poses to be with respect to world frame 

        Args:
            part_poses (List): Information on parts detected by the camera
            sensor_pose (Pose): Pose of the camera in the world frame.
        """
        
        self.parts = []
        self.poses = []
        for part_pose in part_poses:

            part_world_pose = self._multiply_pose(sensor_pose, part_pose.pose)
            self.parts.append(Part(part_color = part_pose.part.color,
                                   part_type = part_pose.part.type))
            self.poses.append(ObjectPose(object_position = part_world_pose.position,
                                        object_orientation = part_world_pose.orientation))
        
        self.sensor_pose = ObjectPose(object_position = sensor_pose.position,
                                      object_orientation= sensor_pose.orientation)

    def __str__(self) -> str:
        """
        Returns a string with the attributes of the PartPoses class

        Returns:
            str: String with all the attributes information
        """
    
        output = "part_poses: \n"+"".join("{}/n{}".format(part.__str__(), pose.__str__()) for part, pose in zip(self.parts, self.poses)) + "\n\t".format(self.sensor_pose.__str__())
        return output

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

        self.declare_parameter('order_id', "0")
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

        self.timed_function = self.create_timer(0.1, self.parse_print)

        # Initiate the poses as None
        self.tray1_poses = None
        self.tray2_poses = None
        self.part1_poses = None
        self.part2_poses = None
        self.order = []


        self.order_id_to_pick = None

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


    def parse_print(self):
        """
        Print all the required information by parsing the objects created.
        Check if all the message flags are disabled and parse flag is enabled
        """
        # print(self.table1_msg, self.table2_msg, self.left_bin_msg, self.right_bin_msg)
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
            output_part = []

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
                            output_part.append(output_part_n)
                            break
                    
            part_string = "".join("{}".format(part) for part in output_part)
            output = "{} \nTray:{} \nPart:\n{}".format(output_order, output_tray, part_string)
            self.get_logger().info('%s' % output)
            
            self.parse_flag = False

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()