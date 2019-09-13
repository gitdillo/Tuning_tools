from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
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

min_start_altitude = 40   # metres
min_home_distance = 100

min_degrees_trim = -10          # we will go this many degrees below 0 pitch trim. Make sure it is NEGATIVE!
max_degrees_trim = 10           # we will go this many degrees above 0 pitch trim. Make sure it is POSITIVE!
trim_degree_step = 2            # we will step so many degrees in our tuning

min_airspeed = 8                # in m/s

waypoint_radius = 10            # this is how close we return to the high waypoint before we consider it hit
waypoint_alt_tolerance = 5      # as with waypoint_radius but for vertical separation

heading_degrees_tolerance = 10  # this is how many degrees off the home bearing we will tolerate before switching off and gliding

throttle_channel = 3            # self explanatory really
throttle_off_pwm = 990  # This value should be low enough to turn off the throttle but not so low as to trigger a possible failsafe





# Internal variables, best left alone
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')    # for use with our logger


def setup_logger(name, log_file, level=logging.INFO):
    """Function setup as many loggers as you want"""

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger

def vector2target(lat1, lon1, lat2, lon2):
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


# This is cleanup. Reset pitch trim, mode and remove channel overrides
def graceful_exit():
  logger.info('Cleanup started.')
  while (not vehicle.mode == original['mode']) or (not vehicle.parameters['TRIM_PITCH_CD'] == original['trim_pitch_cd']) or (not vehicle.channels.overrides == {}):
    vehicle.mode = original['mode']
    vehicle.parameters['TRIM_PITCH_CD'] = original['trim_pitch_cd']
    vehicle.channels.overrides = {}
    logger.info('Commands for restoring mode, trim and channel overrides sent')
    sleep(.2)
  logger.info('Mode, trim and channel overrides have been restored. Exiting.')
  sys.exit(1)   # exit 1 indicates successful, premature exit


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


def goto_point(tgt_lat, tgt_lon, tgt_alt):
  ''' Uses GUIDED to return to the high point. BLOCKS till done '''
  # Set the target location in global-relative frame
  a_location = LocationGlobalRelative(tgt_lat, tgt_lon, tgt_alt)
  vehicle.simple_goto(a_location)
  vehicle.mode = VehicleMode("GUIDED")  # just in case the simple goto didn't switch modes properly
  vehicle.channels.overrides = {}  # disable channel overrides, just in case
  info_string = 'Heading for point ' + str(a_location)
  logger.info(info_string)
  while (not vector2target(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, a_location.lat, a_location.lon)['distance'] < waypoint_radius) or (not abs(vehicle.location.global_relative_frame.alt - a_location.alt) < waypoint_alt_tolerance):
    sleep(1)
  info_string='Reached point ' + str(vehicle.location.global_relative_frame)
  logger.info(info_string)


def gliding_config(trim_degrees):
  '''
  Puts vehicle in STABILIZE, turns off throttle and trims according to argument
  '''
  while (not vehicle.mode == VehicleMode("STABILIZE")) or (vehicle.channels.overrides == {}) or (not vehicle.parameters['TRIM_PITCH_CD'] == trim_degrees * 100):
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.channels.overrides['3'] = throttle_off_pwm
    vehicle.parameters['TRIM_PITCH_CD'] = trim_degrees * 100



  


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
    sleep(1)  # perhaps this will help with: SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
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
  #sys.exit('TODO: do something less idiotic than exiting if initial conditions are not met')
logger.info('Initial conditions are met, script will now take over the vehicle.')


# !!! Milestone !!!
# Reaching here means we have passed all tests and are ready to take control of the vehicle

# First, record the states of things we are going to touch so we can leave them where we found them
original = {'mode': vehicle.mode.name, 'trim_pitch_cd': vehicle.parameters['TRIM_PITCH_CD'], 'altitude': vehicle.location.global_relative_frame.alt,
            'latitude': vehicle.location.global_relative_frame.lat, 'longitude': vehicle.location.global_relative_frame.lon}


# Here we loop over the actual steps of pitch changes. The mechanism is as follows:
# Go to high point
# Set waypoint for home
# wait till heading there within "heading_degrees_tolerance"
# switch to STAB, override throttle to "throttle_off_pwm"
pitch_values_degrees = range(min_degrees_trim, max_degrees_trim, trim_degree_step)
for i in range(len(pitch_values_degrees)):
  if not i == 0:     # if this is the first time, we do not need to go to original location, we are already there
    goto_point(original['latitude'],
               original['longitude'], original['altitude'])
  
  







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
# This is how we use channel overrides https://dronekit-python.readthedocs.io/en/latest/examples/channel_overrides.html
##################################
# Set Ch2 override to 200 using indexing syntax
# vehicle.channels.overrides['2'] = 200
# Set Ch3, Ch4 override to 300,400 using dictionary syntax"
# vehicle.channels.overrides = {'3': 300, '4': 400}
# Clearing an override:
# vehicle.channels.overrides['3'] = None


##################################
# This is how you use GUIDED (https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html#guided-mode-how-to-send-commands)
##################################
# Set mode to guided - this is optional as the goto method will change the mode if needed.
# vehicle.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
# a_location = LocationGlobalRelative(lat, lon, alt)
# vehicle.simple_goto(a_location)

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








