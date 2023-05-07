# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

parameter_file = os.path.join(
                    get_package_share_directory('rwa4_group13'),
                    'config',
                    'order.yaml')

# this function is needed
def generate_launch_description():
    '''
    Function that  returns a LaunchDescription object

    Returns:
        LaunchDescription: LaunchDescription object
    '''
    launch_description = LaunchDescription() # instantiate a Launchdescription object

    subscriber_node = Node( # declare your Node
                        package="rwa4_group13", # package name
                        executable="listener", # executable as set in setup.py
                        parameters=[parameter_file]
                        )
    # start_node = Node( # declare your Node
    #                     package="rwa4_group13", # package name
    #                     executable="start_comp", # executable as set in setup.py
    #                     parameters=[parameter_file]
    #                     )
    launch_description.add_action(subscriber_node) # add each Node to the LaunchDescription object
    # launch_description.add_action(start_node) # add each Node to the LaunchDescription object
    return launch_description # return the LaunchDescription object 