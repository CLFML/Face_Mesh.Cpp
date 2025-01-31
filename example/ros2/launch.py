from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "face_detector", package='face_detector', executable='face_detector_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "face_mesh", package='face_mesh', executable='face_mesh_node', output='screen'),
        launch_ros.actions.Node(
            namespace= "face_mesh", package='face_mesh', executable='face_mesh_viewer', output='screen'),
    ])