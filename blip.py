from dronekit import connect
from math import radians, degrees, cos, sin, asin, sqrt, atan2
from time import sleep
import sys
import threading

# TODO These guys and more will come from a config file in the future
connection_string='/dev/ttyAMA0'
baud_rate=921600
script_control_channel = '6'  # needs to be a string rather than a number as this is how it is handled in vehicle.channels['whatever']
control_channel_low_boundary = 1100   # values below this mean we are in the channel's LOW state  
control_channel_high_boundary = 1900  # values above this mean we are in the channel's HIGH state





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

# TODO This callback will actually start and stop the tuning process
def channels_callback(self, attr_name, value):
  if value[script_control_channel] >= control_channel_high_boundary:
    print('HIGH ' + str(value[script_control_channel]))
    return
  if value[script_control_channel] <= control_channel_low_boundary:
    print('LOW ' + str(value[script_control_channel]))
    return

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


def channel_watcher():
  '''
  This will be used as a tiny thread whose only purpose
  it is to monitor the control channel and shut the script down
  if there is a change in state
  '''
  while (control_channel_state()): sleep(.2)
  print('Oh dear! Control channel no longer HIGH. Prep for exit...')
  graceful_exit()


# TODO This is cleanup. Reset pitch trim and change to some human mode
def graceful_exit():
  print('Shutting down')



# Connect to vehicle
sys.stdout.write('Connecting to vehicle...')
sys.stdout.flush()
vehicle = connect(connection_string, wait_ready=True, baud=baud_rate)
print('OK')


# Check our control channel is in the LOW state
# This is a prerequisite for the tuning to start
sys.stdout.write('Checking control channel is at LOW state...')
while control_channel_state():
  print('\nChannel ' + script_control_channel + ': ' +
        str(vehicle.channels[script_control_channel]) + '. Must be <=' + str(control_channel_low_boundary))
  sleep(2)
print('OK')

# If we reach here, we are good to go on the HIGH signal from the control channel
# so twiddle thumbs till we get it
print('Waiting for start signal from control channel...')
while (not control_channel_state()):
  sleep(.2)




# If we reach here, we got a HIGH signal on the control channel
# Start channel monitoring thread
print('Control channel start signal received, starting monitoring thread')
monitoring_thread = threading.Thread(target=channel_watcher)
monitoring_thread.start()






# This is now you add listeners
#vehicle.add_attribute_listener('channels', channels_callback)
#vehicle.remove_attribute_listener('channels', channels_callback)








