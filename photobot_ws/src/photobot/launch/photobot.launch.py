from os import environ as env
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    # Запуск узла лидара Sick Scan
    lidar = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        remappings=[ ('/sick_nav_350/scan', '/scan'), ], 
        parameters=[{
            'hostname': env["LIDAR_IP_ADDRESS"],
            'scanner_type': env["LIDAR_SCANNER_TYPE"],
        }],
        arguments=[]
    )
    ld.add_action(lidar)
       
    # Запуск узла USB-камеры
    ld.add_action(DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('usb_camera_driver') + '/config/camera.yaml'))
    
    usb_camera = Node(
        package='usb_camera_driver',
        executable='usb_camera_driver_node',
        namespace='/camera',
        parameters=[{
            "camera_calibration_file": LaunchConfiguration('camera_calibration_file'),
            "camera_id": int(env["USB_CAMERA_DEVICE_NUMBER"])
        }]
    )
    ld.add_action(usb_camera)

    # Запуск узла GPS-приемника NovaTel
    gps = launch_ros.actions.ComposableNodeContainer(
        name='novatel_gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='novatel_gps_driver',
                plugin='novatel_gps_driver::NovatelGpsNode',
                name='novatel_gps',
                parameters=[{
                    'connection_type': 'serial',
                    'device': env["GPS_DEVICE_PATH"],
                    'verbose': True,
                    'publish_novatel_positions': True,
                    'publish_novatel_psrdop2': True,
                    'publish_novatel_velocity': True,
                    'frame_id': '/gps'
                }]
            )
        ],
        output='screen'
    )
    ld.add_action(gps)

    # Запуск узла IMU-датчика XSens MTI-1
    imu = Node(
        package='ros2_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[],
        arguments=[]
    )
    ld.add_action(imu)

    # Запуск узла двигателей OnDrive
    motors = Node(
        package='odrive_can',
        executable='odrive_can_node',
        namespace='odrive_axis0',
        name='can_node',
        output='screen',
        parameters= [{
            'node_id': 0,
            'interface': env["ODRIVE_CAN_INTERFACE"]

        }],
        arguments=[]
    )

    ld.add_action(motors)

    # заупуск графического интерфейса RQT
    rqt = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui",
    )

    ld.add_action(rqt)

    return ld