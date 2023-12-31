import carla
import os
from ROAR.utilities_module.data_structures_models import Location, Transform, Rotation


def convert_transform_from_str_to_list(str_transform: str):
    coordinates = str_transform.split(",")
    return [float(i.strip()) for i in coordinates]


def get_coords_from_str(a):
    transform = convert_transform_from_str_to_list(a)
    return [transform[0], transform[2]]


def get_coords_from_str_lines(lines):
    xy_coords_list = []

    for line in lines:
        coords = line.split(',')
        x_coord = float(coords[0].strip())
        y_coord = float(coords[2].strip())
        x_coord = int(x_coord)
        y_coord = int(y_coord)
        xy_coords_list.append([x_coord, y_coord])

    return xy_coords_list


def convert_rotation_from_agent_to_source(roll, pitch, yaw) -> carla.Rotation:
    roll, pitch, yaw = roll, pitch, -yaw
    if yaw <= 0:
        yaw = yaw + 270
    else:
        yaw = yaw - 90
    return carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)


def convert_location_from_agent_to_source(x, y, z) -> carla.Location:
    return carla.Location(x=x, y=z, z=y)


def convert_transform_from_str_to_source(line):
    x, y, z, roll, pitch, yaw = convert_transform_from_str_to_list(line)
    return carla.Transform(convert_location_from_agent_to_source(x, y, z),
                           convert_rotation_from_agent_to_source(roll, pitch, yaw))


def convert_location_from_str_to_agent(line):
    x, y, z, roll, pitch, yaw = convert_transform_from_str_to_list(line)
    return Location(x=x, y=y, z=z)


def convert_transform_from_str_to_agent(line):
    x, y, z, roll, pitch, yaw = convert_transform_from_str_to_list(line)
    return Transform(location=Location(x=x, y=y, z=z),
                     rotation=Rotation(roll=roll, pitch=pitch, yaw=yaw))


def convert_transform_from_agent_to_source(transform: Transform) -> carla.Transform:
    return carla.Transform(convert_location_from_agent_to_source(transform.location.x,
                                                                 transform.location.y,
                                                                 transform.location.z),
                           convert_rotation_from_agent_to_source(transform.rotation.roll,
                                                                 transform.rotation.pitch,
                                                                 transform.rotation.yaw))


def is_dev_mode():
    return os.getenv("ROAR") == "DEV"
