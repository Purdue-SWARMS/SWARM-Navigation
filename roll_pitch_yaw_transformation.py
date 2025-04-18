import airsimneurips
import numpy as np
import math

"""
This file contains the functions to convert coordinates in the NED system to coordinates in the ENU system, and vice
versa.
Doing so is necessary as the drone's coordinate system (NED) is different from the world's coordinate system (ENU).
This allows us to determine the position of the drone with respect to its surroundings for efficient navigation. 
"""

# This establishes a connection between our Python script and the simulation environment.
client = airsimneurips.MultirotorClient()
client.confirmConnection()
print('Connection confirmed')  # Confirms that connection to the simulation is successful

def quaternion_to_yaw(q):
    """
        Convert a quaternion to yaw (rotation around Z-axis).
        The formula is:
        yaw = arctan2 (2(wz+xy),1−2(y^2 + z^2))

    """
    # AirSim uses (w, x, y, z) order for quaternions. Convert using the formula provided
    yaw = np.arctan2(2.0 * (q.w_val * q.z_val + q.x_val * q.y_val),
                     1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val))
    return yaw

def get_ned_to_world_yaw(client, drone_name="drone_1"):
    """
        Retrieves the drone's position and orientation from AirSimNeurIPS functions.
        Computes the transformation matrix from NED to world coordinates.
    """
    # Get the multirotor state
    state = client.getMultirotorState(vehicle_name = drone_name)

    # Extract position in NED coordinates
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val

    # Extract orientation as a quaternion
    orientation = state.kinematics_estimated.orientation
    # print(orientation)

    kinematics = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
    # print(kinematics)

    # Convert quaternion to yaw angle
    yaw = quaternion_to_yaw(orientation)
    print(yaw)

    # Check the yaw value
    print(f" Rotated by (yaw): {np.degrees(yaw)}")

    # Compute the rotation matrix components
    cos_val = np.cos(yaw)
    sin_val = np.sin(yaw)

    # Define the rotation matrix (flipping the z-axis)
    R = np.array([
        [cos_val, -sin_val, 0],
        [sin_val, cos_val, 0],
        [0, 0, -1]
    ])

    # Build the full 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R   # Insert the rotation matrix in the top-left
    T[:3, 3] = [x, y, z]    # Insert the vector in the rightmost column

    return R, T


def quaternion_to_pitch(q):
    """
            Convert a quaternion to pitch (rotation around Y-axis).
            The formula is:
            pitch = arcsin (2.0 * (w * y - z * x))

    """
    # AirSim uses (w, x, y, z) order for quaternions. Convert using the formula provided
    pitch = np.arcsin(2.0 * (q.w_val * q.y_val - q.z_val * q.x_val))
    return pitch

def get_ned_to_world_pitch(client, drone_name = "drone_1"):
    """
            Retrieves the drone's position and orientation from AirSimNeurIPS functions.
            Computes the transformation matrix from NED to world coordinates.
    """
    # Get the multirotor state
    state = client.getMultirotorState(vehicle_name=drone_name)

    # Extract position in NED coordinates
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val

    # Extract orientation as a quaternion
    orientation = state.kinematics_estimated.orientation
    # print(orientation)

    kinematics = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
    # print(kinematics)

    # Convert quaternion to pitch angle
    pitch = quaternion_to_pitch(orientation)
    print(pitch)

    # Check the pitch value
    print(f" Rotated by (pitch): {np.degrees(pitch)}")

    # Compute the rotation matrix components
    cos_val = np.cos(pitch)
    sin_val = np.sin(pitch)

    # Define the rotation matrix (flipping the z-axis)
    R = np.array([
        [cos_val, 0, sin_val],
        [0, 1, 0],
        [-sin_val, 0, cos_val]
    ])

    # Build the full 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R  # Insert the rotation matrix in the top-left
    T[:3, 3] = [x, y, z]  # Insert the vector in the rightmost column

    return R, T

def quaternion_to_roll(q):
    """
            Convert a quaternion to roll (rotation around X-axis).
            The formula is:
            roll = arctan2 (2(wx+yz),1−2(x^2 + y^2))

    """
    # AirSim uses (w, x, y, z) order for quaternions. Convert using the formula provided
    roll = np.arctan2(2.0 * (q.w_val * q.x_val + q.y_val * q.z_val), 1.0 - 2.0 * (q.x_val * q.x_val + q.y_val * q.y_val))
    return roll

def get_ned_to_world_roll(client, drone_name = "drone_1"):
    """
            Retrieves the drone's position and orientation from AirSimNeurIPS functions.
            Computes the transformation matrix from NED to world coordinates.
    """
    # Get the multirotor state
    state = client.getMultirotorState(vehicle_name=drone_name)

    # Extract position in NED coordinates
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val

    # Extract orientation as a quaternion
    orientation = state.kinematics_estimated.orientation
    # print(orientation)

    kinematics = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
    # print(kinematics)

    # Convert quaternion to roll angle
    roll = quaternion_to_roll(orientation)
    print(roll)

    # Check the roll value
    print(f" Rotated by (roll): {np.degrees(roll)}")

    # Compute the rotation matrix components
    cos_val = np.cos(roll)
    sin_val = np.sin(roll)

    # Convert radians to degrees
    # cos_val = np.degrees(cos_val)
    # sin_val = np.degrees(sin_val)

    # Define the rotation matrix (flipping the z-axis)
    R = np.array([
        [1, 0, 0],
        [0, cos_val, -sin_val],
        [0, sin_val, cos_val]
    ])

    # Build the full 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R  # Insert the rotation matrix in the top-left
    T[:3, 3] = [x, y, z]  # Insert the vector in the rightmost column

    return R, T


def roll_pitch_yaw_NTW_transform(client, drone_name = "drone_1"):
    """
            Retrieves the drone's roll, pitch, and yaw from the get functions.
            Computes the transformation matrix from NED to world coordinates.
    """

    # Get the multirotor state
    state = client.getMultirotorState(vehicle_name=drone_name)

    # Extract position in NED coordinates
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val

    # Call the get roll, pitch, and way functions to get the rotation matrices
    roll_rotation, temp_var = get_ned_to_world_roll(client, drone_name)
    pitch_rotation, temp_var = get_ned_to_world_pitch(client, drone_name)
    yaw_rotation, temp_var = get_ned_to_world_yaw(client, drone_name)

    # Compute the matrix multiplication of rotation matrices
    NTW_rotation = yaw_rotation @ pitch_rotation @ roll_rotation
    # print(f"Rotation matrix :")
    # print(NTW_rotation)

    # Create the transformation matrix
    T = np.eye(4)
    T[:3, :3] = NTW_rotation  # Insert the rotation matrix in the top-left
    T[:3, 3] = [x, y, z]  # Insert the vector in the rightmost column

    return T

def roll_pitch_yaw_WTN_transform(client, drone_name = "drone_1"):
    """
            Computes the inverse transformation matrix of NED to world.
            Effectively computes the world to NED transformation matrix.
    """

    # Cal the NED to world transformation function
    transform_matrix = roll_pitch_yaw_NTW_transform(client, drone_name)

    # Extract the rotation matrix
    rotation_matrix = transform_matrix[:3, :3]

    # Compute the inverse rotation matrix
    inv_rotation = np.linalg.inv(rotation_matrix)

    # Create the transformation matrix
    T = transform_matrix
    T[:3, :3] = inv_rotation  # Insert the rotation matrix in the top-left

    return T





# Test the functions

# Call the NED to world and world to NED functions to get the initial transformation matrices
transform = roll_pitch_yaw_NTW_transform(client, drone_name="drone_1")
inv_transform = roll_pitch_yaw_WTN_transform(client, drone_name="drone_1")
print("NED to World Transformation Matrix before takeoff:")
print(transform)
print(inv_transform)

# Enable the API
client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
client.simStartRace()  # Start the race
client.takeoffAsync(vehicle_name="drone_1").join()

# Move by specific roll, pitch, and yaw to compute the transformation matrices after rotation.
client.moveByRollPitchYawZAsync(0, 0, np.radians(-90), -10, duration=10, vehicle_name="drone_1").join()
client.moveByRollPitchYawZAsync(np.radians(90), np.radians(45), np.radians(-90), -10, duration=10, vehicle_name="drone_1").join()
transform = roll_pitch_yaw_NTW_transform(client, drone_name="drone_1")
inv_transform = roll_pitch_yaw_WTN_transform(client, drone_name="drone_1")
print(transform)
print(inv_transform)

