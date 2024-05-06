from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description = LaunchDescription()

    # control_config_arg = DeclareLaunchArgument(
    #     name="controller_config",
    #     default_value=PathJoinSubstitution([
    #         FindPackageShare("omnidirectional_controllers"), "config/position_controller_config.yaml"
    #     ])
    # )
    # #
    # #
    # description.add_action(control_config_arg)

    description.add_action(
        Node(
            executable="ocam",
            name="ocam",
            package="ocam",
            output="screen",
            emulate_tty="True"
        )
    )

    return description

# <!-- ... -->
# <launch>
#   <node pkg="ocam" type="ocam" name="ocam" output="screen">
#     <param name="resolution"    value="1"/> <!-- 0: 1280x960, 1: 1280x720, 2: 640x480, 3: 640x360 -->
#     <param name="frame_rate"    value="30"/>
#     <param name="exposure"      value="100"/>
#     <param name="gain"          value="50"/>
#     <param name="wb_blue"       value="200"/>
#     <param name="wb_red"        value="160"/>
#     <param name="auto_exposure" value="true"/>
#     <param name="show_image"    value="false"/>
#   </node>
# </launch>
