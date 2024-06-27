import carla
import math

from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle
from transforms3d.euler import euler2quat



def snapToGround(world, location, blueprint):
	"""Mutates @location to have the same z-coordinate as the nearest waypoint in @world."""
	waypoint = world.get_map().get_waypoint(location)
	# patch to avoid the spawn error issue with vehicles and walkers.
	z_offset = 0
	if blueprint is not None and ("vehicle" in blueprint or "walker" in blueprint):
		z_offset = 0.5

	location.z = waypoint.transform.location.z + z_offset
	return location


def scenicToCarlaVector3D(x, y, z=0.0):
	# NOTE: Used for velocity, acceleration; superclass of carla.Location
	z = 0.0 if z is None else z
	return carla.Vector3D(x, -y, z)


def scenicToCarlaLocation(pos, z=None, world=None, blueprint=None):
	if z is None:
		assert world is not None
		return snapToGround(world, carla.Location(pos.x, -pos.y, 0.0), blueprint)
	return carla.Location(pos.x, -pos.y, z)


def scenicToCarlaRotation(heading):
	yaw = math.degrees(-heading) - 90
	return carla.Rotation(yaw=yaw)


def scenicSpeedToCarlaVelocity(speed, heading):
	currYaw = scenicToCarlaRotation(heading).yaw
	xVel = speed * math.cos(currYaw)
	yVel = speed * math.sin(currYaw)
	return scenicToCarlaVector3D(xVel, yVel)


def carlaToScenicPosition(loc):
	return Vector(loc.x, -loc.y)

def carlaToScenicElevation(loc):
	return loc.z

def carlaToScenicHeading(rot):
	return normalizeAngle(-math.radians(rot.yaw + 90))

def carlaToScenicAngularSpeed(vel):
	return -math.radians(vel.y)


_scenicToCarlaMap = {
	"red": carla.TrafficLightState.Red,
	"green": carla.TrafficLightState.Green,
	"yellow": carla.TrafficLightState.Yellow,
	"off": carla.TrafficLightState.Off,
	"unknown": carla.TrafficLightState.Unknown,
}

def scenicToCarlaTrafficLightStatus(status):
	return _scenicToCarlaMap.get(status, None)

def carlaToScenicTrafficLightStatus(status):
	return str(status).lower()



def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)

def carla_rotation_to_ros_quaternion_custom(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = euler2quat(roll, pitch, yaw)
    return [quat[1], quat[2], quat[3], quat[0]]

def create_engage_message(engage): # TODO: maybe we need to integrate this with ros instead of sending a message
	message =  f'ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage "engage: {engage}" -1'
	return message

def create_goal_message(x, y, z, qx, qy, qz, qw):
	message =  '''ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: '''
	message += "{x: " + f'{x}, y: ' + f'{y}, z: ' f'{z}' 
	message += "}, orientation: " 
	message += '{x: ' + f'{qx}, y: ' + f'{qy}, z: ' + f'{qz}, w: ' + f'{qw}'
	message += '''}}} ' '''
	return message

def create_initial_position_message(x, y, z, qx, qy, qz, qw):
	# TODO: create a cleaner method. 
	# TODO: the conversion euler angles to quaternions is not ok yet
	message = "ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: " 
	message += '"map"}, pose: { pose: {position: {' 
	message += f"x: {x}, y: {y}, z: {z}" 
	message += "}, orientation: " 
	message += '{x: ' + f'{qx}, y: ' + f'{qy}, z: ' + f'{qz}, w: ' + f'{qw}'
	message += '}}, } }' + "'" 
	return message

def set_emergency_stop(status):

	message = f'ros2 service call /api/external/set/emergency tier4_external_api_msgs/srv/SetEmergency "emergency: {status}"'
	return message