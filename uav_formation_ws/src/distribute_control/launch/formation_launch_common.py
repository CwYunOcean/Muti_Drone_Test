"""Shared naming and parameter helpers for formation launch files."""


def _validate_drone_id(drone_id: int) -> int:
    drone_id = int(drone_id)
    if drone_id < 1:
        raise ValueError('drone_id must be a positive 1-based integer')
    return drone_id


def drone_prefix(drone_id: int) -> str:
    """Return the canonical ROS namespace for one formation vehicle."""
    return f'/drone_{_validate_drone_id(drone_id)}'


def formation_topics(drone_id: int) -> dict[str, str]:
    """Return canonical per-drone topic prefixes and control topics."""
    prefix = drone_prefix(drone_id)
    return {
        'prefix': prefix,
        'fmu_in_prefix': f'{prefix}/fmu/in',
        'fmu_out_prefix': f'{prefix}/fmu/out',
        'virtual_w': f'{prefix}/virtual_w',
        'virtual_w1': f'{prefix}/virtual_w1',
        'virtual_w2': f'{prefix}/virtual_w2',
        'debug_phi': f'{prefix}/phi',
        'actual_path': f'{prefix}/viz/actual_path',
        'drone_model': f'{prefix}/viz/drone_model',
    }


def formation_topic_prefix(drone_id: int) -> str:
    """Compatibility alias for launch files that only need the base prefix."""
    return drone_prefix(drone_id)


def formation_grid_origin(drone_id: int, total_uavs: int, spacing: float) -> tuple[float, float]:
    """Return the centered indoor SITL spawn offset for a 1-based drone id."""
    drone_id = _validate_drone_id(drone_id)
    total_uavs = int(total_uavs)
    if total_uavs < 1 or drone_id > total_uavs:
        raise ValueError('drone_id must be within the formation size')

    import math

    columns = max(1, int(math.ceil(math.sqrt(total_uavs))))
    rows = int(math.ceil(total_uavs / columns))
    row = (drone_id - 1) // columns
    column = (drone_id - 1) % columns
    return (
        column * float(spacing) - (columns - 1) * float(spacing) / 2.0,
        row * float(spacing) - (rows - 1) * float(spacing) / 2.0,
    )
