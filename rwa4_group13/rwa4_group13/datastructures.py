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
        if kwargs:
            self.quadrant = kwargs["part_quadrant"] if "part_quadrant" in kwargs else None
            self.part_bin = kwargs["part_bin"] if "part_bin" in kwargs else None
    
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

    def __init__(self, tray_poses, sensor_pose, tray_table) -> None:
        """
        Constructs all the necessary attributes for the TrayPoses object
        Converts tray poses to be with respect to world frame 

        Args:
            tray_poses (list): Information on trays that are detected by the camera
            sensor_pose (Pose): Pose of the camera in the world frame
            tray_table (str): kts1/kts2, The table where tray is placed 
        """
        
        self.ids = []
        self.poses = []
        self.tray_tables = []
        for tray_pose in tray_poses:

            self.ids.append(tray_pose.id)
            tray_world_pose = self._multiply_pose(sensor_pose, tray_pose.pose)
            self.poses.append(tray_world_pose)
            self.tray_tables.append(tray_table)
        
        self.sensor_pose = sensor_pose

class PartPoses(WorldFrame):
    """
    Class to instantiate PartPoses attributes

    Args:
        WorldFrame (Class): Used to convert part poses to be with respect to world frame
    """

    def __init__(self, part_poses, sensor_pose, part_bin) -> None:
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
                                   part_type = part_pose.part.type,
                                   part_bin = part_bin))
            self.poses.append(part_world_pose)
        
        self.sensor_pose = sensor_pose

class OrderActionParams():
    """
    Class to instantiate OrderActionParams attributes

    Methods
    -------
    add_parts(part_type, part_color, part_quadrant, part_bin, part_pose)
        Add part information in the parts list 

    """
    
    def __init__(self, tray_id, tray_table, tray_pose, agv_number, destination) -> None:
        """Constructs all the necessary attributes for the OrderActionParams object

        Args:
            tray_id (int): The tray to use to build the kit
            tray_table (str): kts1/kts2, The table where tray is placed 
            tray_pose (Pose): Tray pose to be with respect to world frame 
            agv_number (str): The AGV to use to build the kit (1, 2, 3, or 4)
            destination (int): The destination to send the AGV when the kit is complete.
        """

        self.tray_id = tray_id
        self.tray_table = tray_table
        self.tray_pose = tray_pose
        self.agv_number = agv_number
        self.destination = destination

        self.parts = []

    def add_parts(self, part_type, part_color, part_quadrant, part_bin, part_pose):
        """
        Append part information in the parts list

        Args:
            part_type (int): The type of the part
            part_color (int): The color of the part
            part_quadrant (int): The location of the part within the tray
            part_bin (str): Left/Right corresponds to which camera sees the part
            part_pose (Pose): Pose of the part in the world frame
        """

        self.parts.append([part_type,
                            part_color,
                            part_quadrant,
                            part_bin,
                            part_pose])


