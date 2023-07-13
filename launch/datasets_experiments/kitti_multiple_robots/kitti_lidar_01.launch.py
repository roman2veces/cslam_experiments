import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    config_path = os.path.join(
        get_package_share_directory("cslam_experiments"), "config/")
    config_file = LaunchConfiguration('config_file').perform(context)

    # Params
    max_nb_robots = int(LaunchConfiguration('max_nb_robots').perform(context))
    dataset = "KITTI" + LaunchConfiguration('sequence').perform(context)
    robot_delay_s = LaunchConfiguration('robot_delay_s').perform(context)  
    launch_delay_s = LaunchConfiguration('launch_delay_s').perform(context)  
    rate = float(LaunchConfiguration('rate').perform(context))

    # Ajust value according to rate
    robot_delay_s = float(robot_delay_s) / rate
    launch_delay_s = float(launch_delay_s) / rate

    # First robot launch file
    robot_file_number = 1
    robot_id = "1"
    namespace = "/r1"

    # CSLAM process
    cslam_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cslam_experiments"),
                         "launch", "cslam", "cslam_lidar.launch.py")),
        launch_arguments={
            "config_path": config_path,
            "config_file": config_file,
            "robot_id": robot_id,
            "namespace": namespace,
            "max_nb_robots": str(max_nb_robots),
            "enable_simulated_rendezvous": LaunchConfiguration('enable_simulated_rendezvous'),
            "rendezvous_schedule_file": os.path.join(get_package_share_directory("cslam_experiments"),
                         "config", "rendezvous", LaunchConfiguration('rendezvous_config').perform(context)),
        }.items(),
    )
    
    bag_file = os.path.join(
        get_package_share_directory("cslam_experiments"), "data",
        dataset + "_" + str(max_nb_robots) + "robots", dataset + "-" + robot_id)
    bag_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cslam_experiments"),
                "launch",
                "sensors",
                "bag_kitti.launch.py",
            )),
        launch_arguments={
            "namespace": namespace,
            "bag_file": bag_file,
            "rate": str(rate)   
        }.items(),
    )

    odom_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cslam_experiments'), 'launch',
                         'odometry', 'rtabmap_kitti_lidar_odometry.launch.py')),
        launch_arguments={
            "namespace": namespace,
            "robot_id": robot_id,
            'log_level': "fatal",
        }.items(),
    )

    # Map storage node
    storage_process = Node(
            namespace=namespace,
            package='cslam_storage',
            executable='cslam_storage.py',
            name='cslam_storage',
            parameters=[LaunchConfiguration('storage_config')]
        )

    # KITTI specific transform
    tf_process = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 velo_link base_link".split(" "),
                      parameters=[])
    tf_process_imu = Node(package="tf2_ros",
                      executable="static_transform_publisher",
                      arguments="0 0 0 0 0 0 imu_link base_link".split(" "),
                      parameters=[])

    # Launch schedule
    schedule = []
    schedule.append(PushLaunchConfigurations())
    schedule.append(cslam_proc)
    schedule.append(PopLaunchConfigurations())
    
    schedule.append(PushLaunchConfigurations())
    schedule.append(odom_proc)
    schedule.append(PopLaunchConfigurations())        
    
    # schedule.append(PushLaunchConfigurations())
    # We need some delays here to allow the robot to get initialized before 
    # launching the bag
    # schedule.append(
    #         TimerAction(period=float(launch_delay_s),
    #                     actions=[bag_proc]))
    # schedule.append(PopLaunchConfigurations())
    
    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process)
    schedule.append(PopLaunchConfigurations())

    schedule.append(PushLaunchConfigurations())
    schedule.append(tf_process_imu)
    schedule.append(PopLaunchConfigurations())

    schedule.append(PushLaunchConfigurations())
    schedule.append(storage_process)
    schedule.append(PopLaunchConfigurations())

    return schedule


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('max_nb_robots', default_value='2'),
        DeclareLaunchArgument('sequence', default_value='00'),
        DeclareLaunchArgument('robot_delay_s', default_value='260', description="Delay between launching each robot. Ajust depending on the computing power of your machine."),
        DeclareLaunchArgument('launch_delay_s', default_value='10', description="Delay between launching the bag and the robot. In order to let the robot initialize properly and not loose the first bag data frames."),
        DeclareLaunchArgument('config_file',
                              default_value='kitti_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('rate', default_value='0.2'),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='false'),
        DeclareLaunchArgument('rendezvous_config', default_value='kitti00_2robots_lidar.config'),
        DeclareLaunchArgument('storage_config',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_storage'),
                                  'config', 'robot_storage.yaml'),
                              description=''),
        OpaqueFunction(function=launch_setup)
    ])
