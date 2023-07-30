import carla


def get_coords_from_str(a):
    a = a.split(',')
    x, y, z, roll, pitch, yaw = map(float, a)
    return [x,z]

def convert_rotation_from_agent_to_source_real(roll, pitch, yaw) -> carla.Rotation:
    roll, pitch, yaw = roll, pitch, -yaw
    if yaw <= 0:
        yaw = yaw + 270
    else:
        yaw = yaw - 90
    return carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)

def extract_xy_coords_from_string(coord_string):
    lines = coord_string.strip().split('\n')
    xy_coords_list = []
    
    for line in lines:
        coords = line.split(',')
        x_coord = float(coords[0].strip())
        y_coord = float(coords[2].strip())
        x_coord = int(x_coord)
        y_coord = int(y_coord)
        xy_coords_list.append([x_coord, y_coord])
    
    return xy_coords_list

def extract_xy_coords_from_array(coord_array):
    xy_coords_list = []
    
    for line in coord_array:
        coords = line.split(',')
        x_coord = float(coords[0])
        y_coord = float(coords[2])
        x_coord = int(x_coord)
        y_coord = int(y_coord)
        xy_coords_list.append([x_coord, y_coord])
    
    return xy_coords_list

def get_coordinates_from_last_line(file_path):
    with open(file_path, "r") as file:
        lines = file.readlines()
    if len(lines) == 0:
        return None
    # overwrite = input("Do you want to overwrite all previoud waypoints? [Y/n]").upper() == "Y"
    # if overwrite:
    #     with open(file_path, "w") as file:
    #         file.write("")
    #     return None
    last_line = lines[-1].strip()
    coordinates = last_line.split(',')
    x, y, z, roll, pitch, yaw = map(float, coordinates)
    # TODO: fix this, when uncommented, it will cause the car to spawn in the air
    return carla.Transform(carla.Location(x=x, y=z, z=y), convert_rotation_from_agent_to_source_real(roll, pitch, yaw))