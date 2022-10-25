from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
     ld = LaunchDescription()
     controller_node_name = 'robo_cleaner_controller'
     print('[launch.py] - loading node ({0}) params from: ({1})'.format(controller_node_name, "not used at the moment"))

     controller_node = Node(
          package = controller_node_name,
          executable = controller_node_name,
          output = 'screen',
          emulate_tty = True,
          parameters = []
        #   parameters = [config]
     )

     ld.add_action(controller_node)
     return ld