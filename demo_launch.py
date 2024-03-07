import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('challenge_2'),
        'config',
        'params.yaml'
        )
    
    talker_node = Node(
        package = 'challenge_2',
        executable = 'signal_generator',
        name = 'signal_generator',
        output = 'screen',
        prefix = ["konsole -e"],
        parameters=[config],
        emulate_tty = True,
    )
    
    listener_node = Node(
        package = 'challenge_2',
        executable = 'reconstruction',
        name = 'reconstruction',
        output = 'screen',
        prefix = ["konsole -e"],
        emulate_tty = True,
    )
    
    rqt_plot_node = Node(
        package = 'rqt_plot',
        executable = 'rqt_plot',
        output = 'screen',
    )
    
    l_d = LaunchDescription([talker_node, listener_node, rqt_plot_node])
    return l_d