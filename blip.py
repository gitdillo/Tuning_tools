from dronekit import connect, VehicleMode
from math import radians, degrees, cos, sin, asin, sqrt, atan2
from time import sleep
import sys
import threading
import logging


# TODO These guys and more will come from a config file in the future
connection_string='/dev/ttyAMA0'
baud_rate=921600

script_control_channel = '6'  # needs to be a string rather than a number as this is how it is handled in vehicle.channels['whatever']
control_channel_low_boundary = 1100   # values below this mean we are in the channel's LOW state  
control_channel_high_boundary = 1900  # values above this mean we are in the channel's HIGH state

min_start_altitude = 40 # metres
min_home_distance = 100




# Internal variables
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')    # for use with our logger


def setup_logger(name, log_file, level=logging.INFO):
    """Function setup as many loggers as you want"""

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger

def vector2target(lon1, lat1, lon2, lat2):
    """
    Calculate the distance and bearing between two
    coordinates specified in decimal degrees
    Mashup from:
    https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371000 # Radius of earth in meters.
    distance = c * r

    # Get the bearing
    bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
    bearing = degrees(bearing)
    bearing = (bearing + 360) % 360

    return {'distance': distance, 'bearing': bearing}


def control_channel_state():
  '''
  Returns True when channel is in HIGH PWM
  False when channel is in LOW PWM
  None otherwise
  Threshold values for HIGH and LOW are in global vars:
  control_channel_low_boundary
  control_channel_high_boundary
  '''
  if vehicle.channels[script_control_channel] >= control_channel_high_boundary: return True
  if vehicle.channels[script_control_channel] <= control_channel_low_boundary:
      return False
  else: return None

# TODO this thread could be nudged by an event instead of checking and sleeping
def channel_watcher():
  '''
  This will be used as a tiny thread whose only purpose
  it is to monitor the control channel and shut the script down
  if there is a change in state
  '''
  while (control_channel_state()): sleep(.2)
  logger.info('Oh dear! Control channel no longer HIGH. Prep for exit...')
  graceful_exit()


# TODO This is cleanup. Reset pitch trim and change to some human mode
def graceful_exit():
  logger.info('Shutting down')

def check_initial_conditions():
  '''
  Checks initial conditions are met before the script takes control of the vehicle
  '''
  current_alt = vehicle.location.global_relative_frame.alt
  if not current_alt >= min_start_altitude:
    err_string = 'Altitude: ' + str(current_alt) + ', lower than minimum of ' + str(min_start_altitude)
    logger.error(err_string)
    return False
  
  if not vehicle.gps_0.fix_type == 3:
    logger.error('GPS error, no 3D fix')
    return False
  
  home_distance = vehicle.location.local_frame.distance_home()
  if not home_distance >= min_home_distance:
    err_string = 'Distance to home: ' + str(home_distance) + ', less than minimum of ' + str(min_home_distance)
    logger.error(err_string)
    return False

  # TODO perhaps also check the following:
  # vehicle.system_status.state == 'ACTIVE'
  # vehicle.parameters['ARSPD_USE'] == 1
  # vehicle.airspeed > something reasonable

  return True




# Start our very own logger
logger = setup_logger('tunning_logger', 'tuning_log.log')


# Connect to vehicle
logger.info('-------- Connecting to vehicle --------')
vehicle = None
while vehicle is None:
  try:
    vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)
  except:
    logger.warning('Retrying vehicle connection')
logger.info('Vehicle connection successful')


# Check our control channel is in the LOW state
# This is a prerequisite for the tuning to start
logger.info('Checking control channel is at LOW state.')
while control_channel_state():
  warning_string = 'Channel ' + script_control_channel + ': ' + str(vehicle.channels[script_control_channel]) + '. Must be <=' + str(control_channel_low_boundary)
  logger.warning(warning_string)
  sleep(2)
logger.info('Control channel at LOW state.')

# If we reach here, we are good to go on the HIGH signal from the control channel
# so twiddle thumbs till we get it
logger.info('Initialisation OK. Waiting for start signal from control channel...')
while (not control_channel_state()):
  sleep(.2)


# If we reach here, we got a HIGH signal on the control channel
# Start channel monitoring thread
logger.info('Control channel start signal received, starting channel monitoring thread')
monitoring_thread = threading.Thread(target=channel_watcher)
monitoring_thread.start()
logger.info('Channel monitoring thread started.')



# Check conditions are met for taking over
logger.info('Checking initial conditions for taking over.')
if not check_initial_conditions():
  # TODO do something less idiotic than exiting if initial conditions are not met
  logger.error('Initial condition checks failed, the script will terminate.')
logger.info('Initial conditions are met, script will now take over the vehicle.')


# !!! Milestone !!!
# Reaching here means we have passed all tests and are ready to take control of the vehicle

# First, record the states of things we are going to touch so we can leave them where we found them
original_mode = vehicle.mode.name
original_trim_pitch = vehicle.parameters['TRIM_PITCH_CD']
original_altitude = vehicle.location.global_relative_frame.alt



# TODO
# Loop across trim range
# Set GUIDED to home at max alt
# Switch to STAB when heading home
# Set throttle to 0
# Change trim
# wait till stable
# start recording airspeed, sink
# NEXT










##################################
# This is now you add listeners
##################################
# def channels_callback(self, attr_name, value):
#   if value[script_control_channel] >= control_channel_high_boundary:
#     print('HIGH ' + str(value[script_control_channel]))
#     return
#   if value[script_control_channel] <= control_channel_low_boundary:
#     print('LOW ' + str(value[script_control_channel]))
#     return

#vehicle.add_attribute_listener('channels', channels_callback)
#vehicle.remove_attribute_listener('channels', channels_callback)








