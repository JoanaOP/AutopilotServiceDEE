import json
import math
import threading
import paho.mqtt.client as mqtt
import time
import dronekit
from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import ssl


def arm():
    """Arms vehicle and fly to aTargetAltitude"""
    print("Basic pre-arm checks")  # Don't try to arm until autopilot is ready
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    # Copter should arm in GUIDED mode

    vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print(" Armed")

def take_off(a_target_altitude, manualControl, waypointControl):
    global state
    vehicle.simple_takeoff(a_target_altitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= a_target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    state = 'flying'
    if manualControl:
        w = threading.Thread(target=flying)
        w.start()

    if waypointControl:
        w = threading.Thread(target=go_to_waypoint_2)
        w.start()


def prepare_command(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,
        0,
        0,  # x, y, z positions (not used)
        velocity_x,
        velocity_y,
        velocity_z,  # x, y, z velocity in m/s
        0,
        0,
        0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0,
        0,
    )  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    return msg
'''
These are the different values for the state of the autopilot:
    'connected' (only when connected the telemetry_info packet will be sent every 250 miliseconds)
    'arming'
    'armed'
    'disarmed'
    'takingOff'
    'flying'
    'returningHome'
    'landing'
    'onHearth'

The autopilot can also be 'disconnected' but this state will never appear in the telemetry_info packet 
when disconnected the service will not send any packet
'''
def get_telemetry_info ():
    global state
    telemetry_info = {
        'lat': vehicle.location.global_frame.lat,
        'lon': vehicle.location.global_frame.lon,
        'heading': vehicle.heading,
        'groundSpeed': vehicle.groundspeed,
        'altitude': vehicle.location.global_relative_frame.alt,
        'battery': vehicle.battery.level,
        'state': state
    }
    return telemetry_info


def send_telemetry_info():
    global external_client
    global sending_telemetry_info
    global sending_topic

    while sending_telemetry_info:
        external_client.publish(sending_topic + "/telemetryInfo", json.dumps(get_telemetry_info()))
        time.sleep(0.25)


def returning():
    global sending_telemetry_info
    global external_client
    global internal_client
    global sending_topic
    global state

    # wait until the drone is at home
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'

def flying():
    global direction
    global go
    speed = 1
    end = False
    cmd = prepare_command(0, 0, 0)  # stop
    while not end:
        go = False
        while not go:
            vehicle.send_mavlink(cmd)
            time.sleep(1)
        # a new go command has been received. Check direction
        if direction == "North":
            cmd = prepare_command(speed, 0, 0)  # NORTH
        if direction == "South":
            cmd = prepare_command(-speed, 0, 0)  # SOUTH
        if direction == "East":
            cmd = prepare_command(0, speed, 0)  # EAST
        if direction == "West":
            cmd = prepare_command(0, -speed, 0)  # WEST
        if direction == "NorthWest":
            cmd = prepare_command(speed, -speed, 0)  # NORTHWEST
        if direction == "NorthEst":
            cmd = prepare_command(speed, speed, 0)  # NORTHEST
        if direction == "SouthWest":
            cmd = prepare_command(-speed, -speed, 0)  # SOUTHWEST
        if direction == "SouthEst":
            cmd = prepare_command(-speed, speed, 0)  # SOUTHEST
        if direction == "Stop":
            cmd = prepare_command(0, 0, 0)  # STOP
        if direction == "RTL":
            end = True




def distanceInMeters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def executeFlightPlan(waypoints_json):
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state



    altitude = 6
    origin = sending_topic.split('/')[1]

    waypoints = json.loads(waypoints_json)

    state = 'arming'
    arm()
    state = 'takingOff'
    take_off(altitude, False, False)
    state = 'flying'


    wp = waypoints[0]
    originPoint = dronekit.LocationGlobalRelative(float(wp['lat']), float(wp['lon']), altitude)

    distanceThreshold = 0.50
    for wp in waypoints [1:]:

        destinationPoint = dronekit.LocationGlobalRelative(float(wp['lat']),float(wp['lon']), altitude)
        vehicle.simple_goto(destinationPoint)

        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters (destinationPoint,currentLocation)

        while dist > distanceThreshold:
            time.sleep(0.25)
            currentLocation = vehicle.location.global_frame
            dist = distanceInMeters(destinationPoint, currentLocation)
        print ('reached')
        waypointReached = {
            'lat':currentLocation.lat,
            'lon':currentLocation.lon
        }

        external_client.publish(sending_topic + "/waypointReached", json.dumps(waypointReached))

        if wp['takePic']:
            # ask to send a picture to origin
            internal_client.publish(origin + "/cameraService/takePicture")

    vehicle.mode = dronekit.VehicleMode("RTL")
    state = 'returningHome'

    currentLocation = vehicle.location.global_frame
    dist = distanceInMeters(originPoint, currentLocation)

    while dist > distanceThreshold:
        time.sleep(0.25)
        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters(originPoint, currentLocation)

    state = 'landing'
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'


def executeFlightPlan2(waypoints_json):
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state



    altitude = 6
    origin = sending_topic.split('/')[1]

    waypoints = json.loads(waypoints_json)
    state = 'arming'
    arm()
    state = 'takingOff'
    take_off(altitude, False, False)
    state = 'flying'
    cmds = vehicle.commands
    cmds.clear()

    #wp = waypoints[0]
    #originPoint = dronekit.LocationGlobalRelative(float(wp['lat']), float(wp['lon']), altitude)
    for wp in waypoints:
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, 0, 0, 0, float(wp['lat']), float(wp['lon']), altitude))
    wp = waypoints[0]
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                0, 0, 0, 0, float(wp['lat']), float(wp['lon']), altitude))
    cmds.upload()

    vehicle.commands.next = 0
    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    while True:
        nextwaypoint = vehicle.commands.next
        print ('next ', nextwaypoint)
        if nextwaypoint == len(waypoints):  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Last waypoint reached")
            break
        time.sleep(0.5)

    print('Return to launch')
    state = 'returningHome'
    vehicle.mode = VehicleMode("RTL")
    while vehicle.armed:
        time.sleep(1)
    state = 'onHearth'


def go_to_waypoint():
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state
    global next_person_waypoint
    global go2
    global end2

    origin = sending_topic.split('/')[1]

    altitude = 3
    end2 = False
    cmd = prepare_command(0, 0, 0)  # stop

    currentLocation = vehicle.location.global_frame

    while not end2:
        go2 = False
        while not go2:
            vehicle.send_mavlink(cmd)
            time.sleep(1)
        # a new go command has been received. Check direction
        # cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,0, 0, 0, 0, float(next_person_waypoint['lat']), float(next_person_waypoint['lon']), altitude)
        print('going to next waypoint')
        vehicle.mode = VehicleMode('GUIDED')
        distanceThreshold = 0.5
        destinationPoint = dronekit.LocationGlobalRelative(float(next_person_waypoint['waypoint']['lat']), float(next_person_waypoint['waypoint']['lon']), altitude)
        # vehicle.simple_goto(destinationPoint)

        heading = float(next_person_waypoint['heading']) * (math.pi / 180)

        cmd = vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b100111000111,
            0,
            0,
            0,  # x, y, z positions (not used)
            0,
            0,
            0,  # x, y, z velocity in m/s
            0,
            0,
            0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            heading,
            0,
        )

        cmd2 = vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b111111111000,
            int(float(next_person_waypoint['waypoint']['lat'])*1e7),
            int(float(next_person_waypoint['waypoint']['lon'])*1e7),
            3,  # x, y, z positions (not used)
            0,
            0,
            0,  # x, y, z velocity in m/s
            0,
            0,
            0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0,
            0,
        )
        vehicle.send_mavlink(cmd2)
        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters(destinationPoint, currentLocation)
        print(str(dist))

        while dist >= distanceThreshold:
            print(str(dist))
            currentLocation = vehicle.location.global_frame
            dist = distanceInMeters(destinationPoint, currentLocation)
            vehicle.send_mavlink(cmd2)
            if(dist >= distanceThreshold):
                time.sleep(0.25)

        print('reached')
        vehicle.send_mavlink(cmd)
        z = threading.Thread(target=calculate_dif_heading)
        z.start()



def go_to_waypoint_2():
    global vehicle
    global internal_client, external_client
    global sending_topic
    global state
    global next_person_waypoint
    global go2
    global end2
    global counter

    speed = 2
    end2 = False

    cmd = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b111111000111,
        0,
        0,
        0,# x, y, z positions (not used)
        0,
        0,
        0,  # x, y, z velocity in m/s
        0,
        0,
        0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0,
        0,
    )

    while not end2:
        go2 = False
        while not go2:
            vehicle.send_mavlink(cmd)
            counter = counter + 1
            if(counter == 300):
                vehicle.mode = dronekit.VehicleMode("RTL")
                state = 'returningHome'
                direction = "RTL"
                go = True
                go2 = True
                end2 = False
                print("returning")
                w = threading.Thread(target=returning)
                w.start()
            time.sleep(1)
        if(counter == 180):
            break
        print('going to next waypoint')
        lat = float(next_person_waypoint['waypoint']['lat'])
        lon = float(next_person_waypoint['waypoint']['lon'])
        lat_drone = vehicle.location.global_frame.lat
        lon_drone = vehicle.location.global_frame.lon
        cos = (lat-lat_drone)/(math.sqrt(math.pow((lat-lat_drone), 2)+math.pow((lon-lon_drone), 2)))
        sin = (lon-lon_drone)/(math.sqrt(math.pow((lat-lat_drone), 2)+math.pow((lon-lon_drone), 2)))
        vel_north = cos*speed
        vel_east = sin*speed
        cmd = prepare_command(vel_north, vel_east, 0)
        count = 0
        distanceThreshold = 0.5
        currentLocation = vehicle.location.global_frame
        destinationPoint = dronekit.LocationGlobalRelative(lat, lon, 3)
        dist = distanceInMeters(destinationPoint, currentLocation)
        # vehicle.send_mavlink(cmd)
        lastdist = dist + 1
        # while dist >= distanceThreshold:
        #     print(str(dist))
        #     currentLocation = vehicle.location.global_frame
        #     dist = distanceInMeters(destinationPoint, currentLocation)
        #     if(count == 4):
        #         count = 0
        #         vehicle.send_mavlink(cmd)
        #     if(dist >= distanceThreshold):
        #         time.sleep(0.25)
        #     if(dist < lastdist):
        #         break
        #     count = count + 1
        #     lastdist = dist

        cmd2 = vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b111111111000,
            int(float(next_person_waypoint['waypoint']['lat']) * 1e7),
            int(float(next_person_waypoint['waypoint']['lon']) * 1e7),
            3,  # x, y, z positions (not used)
            0,
            0,
            0,  # x, y, z velocity in m/s
            0,
            0,
            0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0,
            0,
        )
        vehicle.send_mavlink(cmd2)
        currentLocation = vehicle.location.global_frame
        dist = distanceInMeters(destinationPoint, currentLocation)
        print(str(dist))

        while dist >= distanceThreshold:
            print(str(dist))
            currentLocation = vehicle.location.global_frame
            dist = distanceInMeters(destinationPoint, currentLocation)
            vehicle.send_mavlink(cmd2)
            if (dist >= distanceThreshold):
                time.sleep(0.25)

        print('reached')

        heading = float(next_person_waypoint['heading']) * (math.pi / 180)

        heading_turn = heading - vehicle.heading

        cmd = vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0,
            0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b100111000111,
            0,
            0,
            0,  # x, y, z positions (not used)
            0,
            0,
            0,  # x, y, z velocity in m/s
            0,
            0,
            0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            heading,
            0,
        )
        vehicle.send_mavlink(cmd)
        z = threading.Thread(target=calculate_dif_heading)
        z.start()








def calculate_dif_heading():
    global internal_client
    global external_client
    global vehicle
    global next_person_waypoint
    global go

    origin = sending_topic.split('/')[1]
    heading = float(next_person_waypoint['heading'])
    headingThreshold = 0.5
    headingDif = abs(float(vehicle.heading) - heading)
    print(str(vehicle.heading))
    while (headingDif > headingThreshold and state != 'returningHome'):
        print(str(headingDif))
        time.sleep(0.25)
        headingDif = abs(float(vehicle.heading) - heading)

    print('heading reached')
    external_client.publish(origin + "/cameraService/takePicture")
    time.sleep(2)
    external_client.publish(sending_topic + "/waypointReached")
    time.sleep(1)



def process_message(message, client):
    global vehicle
    global direction
    global next_person_waypoint
    global go
    global sending_telemetry_info
    global sending_topic
    global op_mode
    global sending_topic
    global state
    global end2
    global go2
    global counter

    print(message.topic)
    splited = message.topic.split("/")
    origin = splited[0]
    command = splited[2]
    sending_topic = "autopilotService/" + origin

    if command == "connect":
        print("Autopilot service connected by " + origin)
        if state == 'disconnected':
            if op_mode == 'simulation':
                connection_string = "tcp:127.0.0.1:5763"
            else:
                connection_string = "/dev/ttyS0"


            vehicle = connect(connection_string, wait_ready=False, baud=115200)

            vehicle.wait_ready(True, timeout=5000)

            print ('Connected to flight controller')
            state = 'connected'

            #external_client.publish(sending_topic + "/connected", json.dumps(get_telemetry_info()))


            sending_telemetry_info = True
            y = threading.Thread(target=send_telemetry_info)
            y.start()
        else:
            print('Autopilot already connected to flight controller')



    if command == "disconnect":
        vehicle.close()
        sending_telemetry_info = False
        state = 'disconnected'
        print('disconnected')


    if command == "takeOff":
        state = 'takingOff'
        w = threading.Thread(target=take_off, args=[5,True,False ])
        w.start()



    if command == "returnToLaunch":
        # stop the process of getting positions
        vehicle.mode = dronekit.VehicleMode("RTL")
        state = 'returningHome'
        direction = "RTL"
        go = True
        go2 = True
        end2 = False
        print("returning")
        w = threading.Thread(target=returning)
        w.start()

    if command == "armDrone":
        state = 'arming'
        arm()

        # the vehicle will disarm automatically is takeOff does not come soon
        # when attribute 'armed' changes run function armed_change
        vehicle.add_attribute_listener('armed', armed_change)
        state = 'armed'

    if command == "disarmDrone":
        vehicle.armed = False
        while vehicle.armed:
            time.sleep(1)
        state = 'disarmed'


    if command == "land":

        vehicle.mode = dronekit.VehicleMode("LAND")
        state = 'landing'
        while vehicle.armed:
            time.sleep(1)
        state = 'onHearth'

    if command == "go":
        direction = message.payload.decode("utf-8")
        print("Going ", direction)
        go = True

    if command == 'executeFlightPlan':
        waypoints_json = str(message.payload.decode("utf-8"))
        w = threading.Thread(target=executeFlightPlan2, args=[waypoints_json, ])
        w.start()

    if command == 'armAndTakeoff':
        state = 'arming'
        arm()
        state = 'takingOff'
        take_off(3, False, True)
        state = 'flying'

    if command == 'goToWaypoint':
        next_person_waypoint_json = str(message.payload.decode("utf-8"))
        origin = sending_topic.split('/')[1]
        go2 = True
        next_person_waypoint = json.loads(next_person_waypoint_json)
        counter = 0

def on_connect(client, userdata, flags, rc):
    external_client.subscribe("+/autopilotService/#", 2)
    internal_client.subscribe("+/autopilotService/#")


def armed_change(self, attr_name, value):
    global vehicle
    global state

    if vehicle.armed:
        state = 'armed'
    else:
        state = 'disarmed'



def on_internal_message(client, userdata, message):
    global internal_client
    process_message(message, internal_client)

def on_external_message(client, userdata, message):
    global external_client
    process_message(message, external_client)

def AutopilotService (connection_mode, operation_mode, external_broker, username, password):
    global op_mode
    global external_client
    global internal_client
    global state
    global counter

    counter = 0

    state = 'disconnected'

    print ('Connection mode: ', connection_mode)
    print ('Operation mode: ', operation_mode)
    op_mode = operation_mode

    # The internal broker is always (global or local mode) at localhost:1884
    internal_broker_address = "localhost"
    internal_broker_port = 1884

    if connection_mode == 'global':
        external_broker_address = external_broker
    else:
        external_broker_address = 'localhost'


    print ('External broker: ', external_broker_address)



    # the external broker must run always in port 8000, in mqqts mosquitto port 8001 and in broker.hivemq.com wss port 8884
    external_broker_port = 8000


    external_client = mqtt.Client("Autopilot_external", transport='websockets')
    # external_client.tls_set('c:\\Users\\joana\\Documents\\uni\\TFG\\mobileAppCertificates\\rootCA.crt')
    # external_client.tls_set(ca_certs='c:\\Users\\joana\\Documents\\uni\\TFG\\mobileAppCertificates\\rootCA.crt', certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS, ciphers=None)
    # external_client.tls_set(ca_certs=None, certfile=None, keyfile=None, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS, ciphers=None)
    # external_client.tls_insecure_set(True)
    if external_broker_address == 'classpip.upc.edu':
        external_client.username_pw_set(username, password)

    external_client.on_connect = on_connect
    external_client.on_message = on_external_message
    external_client.connect(external_broker_address, external_broker_port)


    internal_client = mqtt.Client("Autopilot_internal")
    internal_client.on_connect = on_connect
    internal_client.on_message = on_internal_message
    internal_client.connect(internal_broker_address, internal_broker_port)

    print("Waiting....")
    external_client.subscribe("+/autopilotService/#", 2)
    internal_client.subscribe("+/autopilotService/#")
    internal_client.loop_start()
    if operation_mode == 'simulation':
        external_client.loop_forever()
    else:
        external_client.loop_start()



if __name__ == '__main__':
    import sys
    connection_mode = sys.argv[1] # global or local
    operation_mode = sys.argv[2] # simulation or production
    username = None
    password = None
    if connection_mode == 'global':
        external_broker = sys.argv[3]
        if external_broker == 'classpip.upc.edu':
            username = sys.argv[4]
            password = sys.argv[5]
    else:
        external_broker = None

    AutopilotService(connection_mode,operation_mode, external_broker, username, password)
