from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, TimeoutError
from math import radians, degrees, cos, sin, asin, sqrt, atan2
from time import sleep, strftime
import sys
import threading
import thread
import logging
import csv

# TODO These guys will come from a config file in the future
connection_string='/dev/ttyAMA0'
baud_rate = 115200

script_control_channel = '6'  # needs to be a string rather than a number as this is how it is handled in vehicle.channels['whatever']
control_channel_boundary = 1500   # values below this mean we are in the channel's LOW state and above we in the HIGH state


min_start_altitude = 40         # we need to be at least this many metres up to start gliding
min_gliding_altitude = 20       # we won't allow the vehicle to glide below this altitude

min_degrees_trim = -10          # we will go this many degrees below 0 pitch trim. Make sure it is NEGATIVE!
max_degrees_trim = 10           # we will go this many degrees above 0 pitch trim. Make sure it is POSITIVE!
trim_degree_step = 2            # we will step so many degrees in our tuning

min_airspeed = 8                # in m/s

waypoint_radius = 10            # this is how close we return to the high waypoint before we consider it hit
waypoint_alt_tolerance = 5      # as with waypoint_radius but for vertical separation

heading_degrees_tolerance = 10  # this is how many degrees off the home bearing we will tolerate before switching off and gliding

throttle_channel = 3            # self explanatory really
throttle_off_pwm = 990          # This value should be low enough to turn off the throttle but not so low as to trigger a possible failsafe

record_filename = 'glide.csv'   # we will save gliding data in here
recording_period = 0.1          # we will take a recording of the glide every so many seconds


# Internal variables, best left alone
formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')    # for use with our logger
runflag = True          # tells the threads we are running
record_flag = False     # tells the recording thread to record
original = None         # this will store the various original values before we start our gliding runs


# -----------------------------
# Functions
# -----------------------------

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


def control_channel_is_low():
  '''
  Returns True when channel is in LOW PWM
  False otherwise
  Threshold values for HIGH and LOW are in global vars:
  control_channel_low_boundary
  control_channel_high_boundary
  '''
  if vehicle.channels[script_control_channel] <= control_channel_boundary: return True
  else: return False


def control_channel_is_high():
  '''
  Kinda stupid function but also kinda handy for the wait_for()
  '''
  return not control_channel_is_low()

# TODO this thread could be nudged by an event instead of checking and sleeping
def channel_watcher():
  '''
  This will be used as a tiny thread whose only purpose
  it is to monitor the control channel and shut the script down
  if there is a change in state
  '''
  while (control_channel_is_high()):
    if not runflag: sys.exit()
    sleep(.2)
  logger.info('Oh dear! Control channel no longer HIGH. Prep for exit...')
  graceful_exit()

# This is cleanup. Reset pitch trim, mode and remove channel overrides
def graceful_exit():
  logger.info('Cleanup started.')
  restore_settings()
  thread.interrupt_main()


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

  # TODO perhaps also check the following:
  # vehicle.system_status.state == 'ACTIVE'
  # vehicle.parameters['ARSPD_USE'] == 1
  # vehicle.airspeed > something reasonable

  return True


def goto_point(tgt_lat, tgt_lon, tgt_alt):
  ''' Uses GUIDED to return to the high point.
  BLOCKS till done '''
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
  BLOCKS till all done!
  '''
  # We *could* use wait_for all these but since it is important they should happen FAST
  # and sending the request multiple times can't hurt, we spam till we get it
  while (not vehicle.mode == VehicleMode("STABILIZE")) or (vehicle.channels.overrides == {}) or (not vehicle.parameters['TRIM_PITCH_CD'] == trim_degrees * 100):
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.channels.overrides['3'] = throttle_off_pwm
    vehicle.parameters['TRIM_PITCH_CD'] = trim_degrees * 100
    sleep(.1)

def heading_towards_home():
  '''
  Will return True if vehicle heading is within "heading_degrees_tolerance" of home
  '''
  tgt_bearing = vector2target(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,
                              vehicle.home_location.lat, vehicle.home_location.lon)['bearing']
  if (tgt_bearing - vehicle.heading) <= heading_degrees_tolerance:
    return True
  else:
    return False
  
def gliding_conditions_not_met():
  '''
  Returns True when conditions for gliding are not OK
  i.e. getting too low or too slow or anything else we might think of later
  '''
  if (vehicle.location.global_relative_frame.alt <= min_gliding_altitude) or (vehicle.airspeed <= min_airspeed):
    return True

def restore_settings():
  '''
  Puts the vehicle back into its initial state
  resetting trim and channel overrides
  '''
  if original is None:
    logger.info('Original values have not been recorded. Nothing to reset.')
    return
  while (not vehicle.mode == original['mode']) or (not vehicle.parameters['TRIM_PITCH_CD'] == original['trim_pitch_cd']) or (not vehicle.channels.overrides == {}):
    vehicle.mode = original['mode']
    vehicle.parameters['TRIM_PITCH_CD'] = original['trim_pitch_cd']
    vehicle.channels.overrides = {}
    logger.info('Commands for restoring mode, trim and channel overrides sent')
    sleep(.2)
  logger.info('Mode, trim and channel overrides have been restored. Exiting.')


def recorder():
  '''
  Thread, saves gliding info into a csv
  '''
  try:
    f = open(record_filename, 'wb')
  except:
    logger.warning('Cannot open ' + str(record_filename) + ' for writing.')
    return
  
  writer = csv.writer(f, delimiter=',')
  writer.writerow(['Time', 'Pitch', 'Lat', 'Lon', 'Alt', 'Airspeed', 'vx', 'vy', 'vz'])
  f.flush()

  while (runflag):
    sleep(.2)
    while (record_flag):
      writer.writerow([strftime("%y%m%d", time.gmtime()), vehicle.parameters['TRIM_PITCH_CD'] / 100, vehicle.location.global_relative_frame.lat,
                       vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt, vehicle.airspeed,
                       vehicle.velocity[0], vehicle.velocity[1], vehicle.velocity[2]])
      sleep(recording_period)

  f.flush
  f.close()
  sys.exit()




# -----------------------------
# Functions no more
# -----------------------------



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
    sleep(2)  # perhaps this will help with: SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
logger.info('Vehicle connection successful')


# Check our control channel is in the LOW state
# This is a prerequisite for the tuning to start
logger.info('Checking control channel is at LOW state.')
vehicle.wait_for(control_channel_is_low)
logger.info('Control channel at LOW state. Continuing.')


# If we reach here, our contorl channel signal is LOW
# so now we can sit back and wait till it goes HIGH
logger.info('Initialisation OK. Waiting for start signal from control channel...')
while (control_channel_is_low()):
  sleep(.2)


# If we reach here, we got a HIGH signal on the control channel
# Start channel monitoring thread
logger.info('Control channel start signal received, starting channel monitoring thread')
monitoring_thread = threading.Thread(target=channel_watcher)
monitoring_thread.start()
logger.info('Channel monitoring thread started.')

# Start recording thread
logger.info('Starting recording thread')
recording_thread = threading.Thread(target=recorder)
recording_thread.start()


# Check conditions are met for taking over
logger.info('Checking initial conditions for taking over.')
if not check_initial_conditions():
  logger.error('Initial condition checks failed, the script will terminate.')
  runflag = False       # this will signal the thread(s) that it's time to go
  # TODO do something less idiotic than exiting if initial conditions are not met
  sys.exit('Termination: initial conditions not met.')

logger.info('Initial conditions are met, script will now take over the vehicle.')


# ########################################3
# !!! Milestone !!!
# Reaching here means we have passed all tests and are ready to take control of the vehicle
# ########################################3
# First, record the states of things we are going to touch so we can leave them where we found them
original = {'mode': vehicle.mode.name, 'trim_pitch_cd': vehicle.parameters['TRIM_PITCH_CD'], 'altitude': vehicle.location.global_relative_frame.alt,
            'latitude': vehicle.location.global_relative_frame.lat, 'longitude': vehicle.location.global_relative_frame.lon}


# Here we loop over the actual steps of pitch changes. The mechanism is as follows:
# Go to high point
# wait till heading there within "heading_degrees_tolerance"
# switch to STAB, override throttle to "throttle_off_pwm"
pitch_values_degrees = range(min_degrees_trim, max_degrees_trim, trim_degree_step)
for i in range(len(pitch_values_degrees)):
  # if this is the first time, we do not need to go to original location, we are already there
  if i == 0:
    logger.info('First time at high point, switching to LOITER')
    vehicle.wait_for_mode(VehicleMode("LOITER"))  # LOITER so we circle at high point
  else:
    logger.info('Heading for high point')
    goto_point(original['latitude'], original['longitude'],
               original['altitude'])  # will block till we reach
  
  # So, now we are circling at high point either in LOITER or GUIDED
  
  # reaching here means we are at the high point, either because it's the first time or because we got there via goto_point()
  # since we are in GUIDED (or LOITER if this is the first time) and circling,
  # we wait till our heading points towards HOME, then switch to STAB and glide
  vehicle.wait_for(heading_towards_home)
  logger.info('Starting Glide with pitch trimmed at ' + str(pitch_values_degrees[i]) + ' degrees.')
  gliding_config(pitch_values_degrees[i])   # takes care of the mode, trim, throttle settings
  record_flag = True                        # start recording
  vehicle.wait_for(gliding_conditions_not_met, interval=0.1)  # wait till we are no longer good for gliding
  record_flag = False                       # stop recording
  restore_settings()  # get rid of changes made by gliding_config()
  logger.info('Glide ended')
  


# Reaching here means we have concluded the gliding runs
# Restore and do something reasonable
restore_settings()
vehicle.wait_for_mode(VehicleMode("RTL"))








######################
# NOTES
######################


  # Set mode to guided - this is optional as the goto method will change the mode if needed.
# vehicle.mode = VehicleMode("GUIDED")

# Set the target location in global-relative frame
# a_location = LocationGlobalRelative(lat, lon, alt)
# vehicle.simple_goto(a_location)





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








