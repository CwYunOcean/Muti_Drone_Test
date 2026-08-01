"""Launch the double-W controller with interactive curve input."""

from formation_sitl_launch import generate_sitl_launch_description


def generate_launch_description():
    return generate_sitl_launch_description('offboard_node_2', 'visualizer_set_node')
