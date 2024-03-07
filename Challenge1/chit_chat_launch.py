from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    talker_node = Node(
        package='basic_comms',
        executable='talker',
        output='screen'
    )
    
    listener_node = Node(
        package='basic_comms',
        executable='listener',
        output='screen'
    )
    
    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen'
    )
    rqt_plot_node = Node( 
        package='rqt_plot',
        executable='rqt_plot',
        output='screen'
    )   
    
    l_d = LaunchDescription([talker_node, listener_node, rqt_graph_node, rqt_plot_node])
    return l_d