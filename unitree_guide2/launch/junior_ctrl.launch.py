from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")
    joy_topic = LaunchConfiguration("joy_topic")
    unitree_address = LaunchConfiguration("unitree_address")
    use_unitree_ble = LaunchConfiguration("use_unitree_ble")

    ble_needed = PythonExpression(
        [
            "'",
            use_unitree_ble,
            "' == 'true' and '",
            input_mode,
            "' != 'keyboard'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_mode",
                default_value="joystick",
                description="junior_ctrl input: keyboard, joystick, or both",
            ),
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument(
                "unitree_address", default_value="98:DA:10:02:9C:4C"
            ),
            DeclareLaunchArgument(
                "use_unitree_ble",
                default_value="true",
                description="Start the Unitree BLE-to-Joy bridge",
            ),
            Node(
                package="unitree_guide2",
                executable="unitree_ble_joy",
                name="unitree_ble_joy",
                output="screen",
                parameters=[
                    {
                        "address": unitree_address,
                        "joy_topic": joy_topic,
                    }
                ],
                condition=IfCondition(ble_needed),
            ),
            Node(
                package="unitree_guide2",
                executable="junior_ctrl",
                name="unitree_gazebo_server",
                output="screen",
                parameters=[
                    {
                        "input_mode": input_mode,
                        "joy.topic": joy_topic,
                    }
                ],
            ),
        ]
    )
