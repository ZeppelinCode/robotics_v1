from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
     ld = LaunchDescription()
     controller_node_name = 'robo_miner_controller'
     longest_sequence_node_name = 'robo_miner_longest_sequence'
    #  config = os.path.join(
    #       get_package_share_directory(node_name),
    #       'config',
    #       'params.yaml'
    #  )

    #  print('[launch.py] - loading node ({0}) params from: ({1})'.format(node_name, config))
     print('[launch.py] - loading node ({0}) params from: ({1})'.format(controller_node_name, "not used at the moment"))
     print('[launch.py] - loading node ({0}) params from: ({1})'.format(longest_sequence_node_name, "not used at the moment"))

     controller_node = Node(
          package = controller_node_name,
          executable = controller_node_name,
          output = 'screen',
          emulate_tty = True,
          parameters = [] # not used at the moment
        #   parameters = [config]
     )

     longest_sequence_node = Node(
          package = longest_sequence_node_name,
          executable = longest_sequence_node_name,
          output = 'screen',
          emulate_tty = True,
          parameters = [] # not used at the moment
        #   parameters = [config]
     )

     ld.add_action(longest_sequence_node)
     ld.add_action(controller_node)
     return ld