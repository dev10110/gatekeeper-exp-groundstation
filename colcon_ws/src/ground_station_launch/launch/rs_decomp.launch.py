
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():


    # realsense
    rs_node = ComposableNode(
            package="realsense2_camera",
            plugin="realsense2_camera::RealSenseNodeFactory",
            parameters=[
                    {"initial_reset": False},
                    #enables
                    {"enable_color": True},
                    {"enable_infra1": False},
                    {"enable_infra2": False},
                    {"enable_accel": False},
                    {"enable_gyro": False},
                    {"pointcloud.enable": True},
                    #qos
                    {"pointcloud.pointcloud_qos": "SENSOR_DATA"},
                    {"color_qos": "SENSOR_DATA"},
                    {"depth_qos": "SENSOR_DATA"},
                    #depth module
                    {"depth_module.profile": "640x480x30"},
                    {"clip_distance": 5.0},
                    {"decimation_filter.enable": True},
                    {"decimation_filter.filter_magnitude": 3},
                    {"depth_module.emitter_enabled": True},
                    {"pointcloud.allow_no_texture_points":True},
                    # color module
                    {"rgb_camera.profile": "640x480x15"},

                ],
            extra_arguments=[{'use_intraprocess_comms': True}],
            )

    # decomp
    decomp_node = ComposableNode(
            package="decomp_ros",
            plugin="decompros::SeedDecomp",
            name="seedDecomp_component",
            parameters=[
                {"fov_v": 58.0},
                {"fov_h": 77.0},
                {"fov_range": 4.0},
                {"fov_obs_ray_range": 2.0},
                {"fov_obs_spacing": 0.25},
                {"fov_obs_skip_first": 1},
                ],
            remappings=[
                ("cloud_in", "/depth/color/points"),
                ]
            )

    # this is the realsense + decomp nodes launched together
    rs_decomp = ComposableNodeContainer(
            name="filter_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions = [
                rs_node, 
                decomp_node,
                ],
            output="screen"
            )
    off_x = "0.0"
    off_y = "0"
    off_z = "0"
    off_roll = "3.14"
    off_pitch = "0"
    off_yaw = "0"
    
    ## define a static tf between vicon and the camera
    static_tf = Node(
            package="tf2_ros", 
            executable="static_transform_publisher",
            arguments=[
               off_x, off_y, off_z, off_yaw, off_pitch, off_roll, "vicon/px4_1/px4_1", "camera_link"]
            )


    return LaunchDescription([
        rs_decomp,
        # static_tf
        ]
        )













