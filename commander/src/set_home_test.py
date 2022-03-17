from pymavlink import mavutil
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14030')
# the_connection = mavutil.mavlink_connection('/dev/ttyACM0')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
def connecting():
    global the_connection
    print("Connecting")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

def set_home_command(lat, lon, alt):    
    global the_connection
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, lat, lon, alt)

def recv_command(message):
    global the_connection
    msg = the_connection.recv_match(type=message, blocking=True)
    return msg

if __name__ == "__main__":
    connecting()
    lat = 37.4139910
    lon = -121.9970255
    alt = 10
    bias = 0
    while True:
        set_home_command(lat+bias, lon, alt)
        # msg = recv_command("COMMAND_ACK")
        # print(msg)
        # msg = recv_command("HOME_POSITION")
        # print(msg)

        msg = recv_command("HEARTBEAT")
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)
            if mode == "RTL":
                break
        # bias = bias + 0.00001 
        # time.sleep(1)
