#!/usr/bin/env python
import math
import serial
import rospy
import numpy as np
from std_msgs.msg import String
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Int32
import csv
import sys
import time

# Author: parker
#teensyWrite_node is subscribed to the 'input_vectors' topic from VectorCalc_node or some equivalent that sends desired [speed, translation,rotation] vectors
#It outputs wheel commands to the Teensy after processing desired vectors from the 'input_vectors' topic
#for logging see the main loop and what to uncomment
#currently compass data comes from an arduino publishing to the compass topic. see compass_node.py
#odometry filters are a work in progress but those are also configurable from the main loop
import topics
from util import to180

ser = serial.Serial('/dev/teensy', 250000,timeout=.1,writeTimeout=.1)
Hz = 30
LENGTH = .66675
WIDTH = .6223
MAX_TURNS = 1#how many turns the planetaries are allowed before re-zeroing
RADIUS = math.sqrt(LENGTH/2*LENGTH/2+WIDTH/2*WIDTH/2)
# psi0 = 43
# psi1 = 133
# psi2 = -133
# psi3 = -43
#for some reason if we have the actual wheel positions our point of rotation does not stay coincident with our translation's axis' perpendicular
#TODO: figure out why^ it may cause issues with odometry later on
psi0 = 45
psi1 = 135
psi2 = -135
psi3 = -45

loggingMatrix=[]

def calcWheel(speed, velocity_vector, theta_dot, wheel_psi, pointTurnAngle, pointTurnDir,angles_only):
    """
    this is the method that actually does the calculations. this takes desired vectors and outputs data for the
    individual wheels. nearly identical to the writeup that Dr. Swanson produced.
    :param speed: desired speed at center of Bender
    :param velocity_vector: desired translational bearing
    :param theta_dot: desired rotational vector
    :param wheel_psi: location on the+-180 scale of each wheel
    :return: wheelString. a string representing desired bearing and speed for the current wheel
    """

    global steering_state
    theta_dot_rad = theta_dot * 0.0174533
    rot_speed = math.fabs(theta_dot_rad) * RADIUS

    if math.fabs(speed) < 0.05:
        if math.fabs(rot_speed) > .001:#point rotation
            steering_state["point_turn"]=True
            steering_state["translation"]=False
            steering_state["regular_turn"]=False
            steering_state["translation_and_rotation"]=False
            steering_state["no_turn"]=False
            wheel_theta = pointTurnAngle
            wheel_speed = theta_dot * 2
            print theta_dot
            if theta_dot > 0:
                wheel_speed *= pointTurnDir
            else:
                wheel_speed *= pointTurnDir
            if angles_only:
                wheel_speed=0
            wheelString = str('B' + str(wheel_theta) + 'S' + str(wheel_speed*-1))
            #wheelString = str('B0' + 'S0')
            return wheelString
        else:
            steering_state["point_turn"] = False
            steering_state["translation"] = False
            steering_state["regular_turn"] = False
            steering_state["translation_and_rotation"] = False
            steering_state["no_turn"] = True
            wheel_theta = velocity_vector
            wheelString = str('B' + str(wheel_theta) + 'S0')
            return wheelString
    else:
        if math.fabs(rot_speed) < .001:  # just translation
            steering_state["point_turn"] = False
            steering_state["translation"] = True
            steering_state["regular_turn"] = False
            steering_state["translation_and_rotation"] = False
            steering_state["no_turn"] = False
            wheel_theta = velocity_vector
            #wheel_speed = speed / 0.0085922
            wheel_speed = speed / 0.0106395  # for yes orange tread
            if angles_only:
                wheel_speed=0
            wheelString = str('B' + str(wheel_theta) + 'S' + str(wheel_speed*-1))
            return wheelString
        else:
            if velocity_vector == 0: #regular_turn like a car
                steering_state["point_turn"] = False
                steering_state["translation"] = False
                steering_state["regular_turn"] = True
                steering_state["translation_and_rotation"] = False
                steering_state["no_turn"] = False
            else:#be fancy
                steering_state["point_turn"] = False
                steering_state["translation"] = False
                steering_state["regular_turn"] = False
                steering_state["translation_and_rotation"] = True
                steering_state["no_turn"] = False
            if (theta_dot < 0):
                angle_vv_rot = wheel_psi - velocity_vector - 90
            else:
                angle_vv_rot = wheel_psi - velocity_vector + 90
            sup_angle = 180 - angle_vv_rot
            if (sup_angle > 180):
                sup_angle = sup_angle - 360
            if (sup_angle < -180):
                sup_angle = sup_angle + 360

            # now convert everything to rads and meters
            sup_angle_rad = sup_angle * 0.0174533

            velocity_vector_rad = velocity_vector * 0.0174533
            wheel_speed = math.sqrt(speed * speed + rot_speed * rot_speed - 2 * speed * rot_speed * math.cos(sup_angle_rad))
            delta_theta = math.asin((math.sin(sup_angle_rad) * rot_speed) / wheel_speed)
            wheel_theta = velocity_vector_rad + delta_theta
            #wheel_speed = wheel_speed / 0.0085922  # for no orange tread
            wheel_speed = wheel_speed / 0.0106395  # for yes orange tread
            wheel_theta = wheel_theta * 57.2958
            if angles_only:
                wheel_speed=0
            wheelString = str('B' + str(int(wheel_theta)) + 'S' + str(int(wheel_speed*-1)))
            #print "wheelString: ",wheelString
            return wheelString

# def getLatestData(numDataSlots):#without dict return type used for
#     """
#     parses the data in the serial buffer sent back from the teensy. Currently I believe we are sending the total
#     distance traveled in meters, the current angle of each planetary motor, and the current speed of each hub motor.
#     NOTE: Teensy should return currentWheelAngle in the main .ino class!
#     All of these returns are from Bender's reference frame, not the world/map frame. This means it will have to be fused
#     with absolute oriantation(compass) data before we can publish to tf. This all depends on what our
#     :return:list data[]
#     """
#     while ser.inWaiting() > 0:
#         toParse = ser.readline()
#         parsed = toParse.split(",")
#         data = []
#         for num in range(0, numDataSlots):
#             try:
#                 data.append(int(parsed[num]))
#             except ValueError:#its a float
#                 data.append(float(parsed[num]))
#         return data

def getLatestData():#with dict lookup. perhaps consider using an assert:ser.inWaiting. Avoid duplicate data=bad map
    """
    Return is in groups of dicts for readability and easier looping when doing our odometry calculations later on.
    easier to loop through 5 lists separately than 1 large list 5 times at different starting positions
    :return:dict 0,1,2,3 represent data for wheels0-wheels3
    """

    while ser.inWaiting() > 0:
        toParse = ser.readline()
        parsed = toParse.split(",")
        print "Length of Parsed: ",len(parsed)
        desired_angle = {
            0: float(parsed[0]),
            1: float(parsed[1]),
            2: float(parsed[2]),
            3: float(parsed[3])
        }
        currentAngle = {
            0: float(parsed[4]),
            1: float(parsed[5]),
            2: float(parsed[6]),
            3: float(parsed[7])
        }
        current_RPM = {
            0: float(parsed[8]),
            1: float(parsed[9]),
            2: float(parsed[10]),
            3: float(parsed[11])
        }
        desired_RPM = {
            0: float(parsed[12]),
            1: float(parsed[13]),
            2: float(parsed[14]),
            3: float(parsed[15])
        }
        speedCheck = {
            0: float(parsed[16]),
            1: float(parsed[17]),
            2: float(parsed[18]),
            3: float(parsed[19])
        }
        deltaTics = {
            0: float(parsed[20]),
            1: float(parsed[21]),
            2: float(parsed[22]),
            3: float(parsed[23])
        }
        deltaAngle = {
            0: float(parsed[24]),
            1: float(parsed[25]),
            2: float(parsed[26]),
            3: float(parsed[27])
        }
        deltaHubRPM = {
            0: float(parsed[28]),
            1: float(parsed[29]),
            2: float(parsed[30]),
            3: float(parsed[31])
        }
        teensyTime=float(parsed[32])

        return desired_angle,currentAngle,current_RPM,desired_RPM,speedCheck,deltaTics,deltaAngle,deltaHubRPM,teensyTime

def checkDir(currentAngle, desired_angle):
    """
    determines if shortest distance to desired angle is tracing right or left and then returns the differnce along with
    a iwheelber representing whether the difference should be added or subtracted to the total_angle. Used to track wraparound of planetary wires
    :param currentAngle: current angle represented as a range of 0to360 going clockwise
    :param desired_angle: desired angle reported from joystick data on the same range as currentAngle
    :return: float,difference.
    """

    difference = 0
    currentAngle = currentAngle%360
    if currentAngle <0:
        currentAngle=currentAngle*-1
    if currentAngle == desired_angle:
        return 0,difference
    elif (currentAngle == 0 and desired_angle <= 180):
        difference = desired_angle-currentAngle
        return 1,difference
    elif (currentAngle == 0 and desired_angle > 180):
        difference = 360 - desired_angle
        return -1,difference
    elif desired_angle == 0 and currentAngle <= 180:
        difference = currentAngle
        return -1,difference
    elif desired_angle == 0 and currentAngle > 180:
        difference = 360-currentAngle
        return 1,difference
    elif currentAngle > desired_angle and currentAngle-desired_angle > 0 and currentAngle-desired_angle<180:
        if desired_angle < currentAngle:
            difference = currentAngle-desired_angle
        else:
            difference = 360 - desired_angle+(currentAngle)
        return -1,difference
    else:
        if desired_angle > currentAngle:
            if(desired_angle-currentAngle>180):
                difference = 360-desired_angle+currentAngle
                difference = difference*-1
            else:
                difference = desired_angle-currentAngle
        else:
            difference = 360-currentAngle+desired_angle
        return 1,difference

def rpm_to_mps(current_RPM):
    """
    converts rpm to mps. currently works for orange wheels
    :param current_RPM:dict
    :return: list
    """
    current_RPM_list = []
    current_mps = {
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }

    for i in range(0,4):
        current_RPM_list.append(current_RPM[i])
    for i in range(0,4):
        current_mps[i]=(current_RPM_list[i]*0.638048)/60
    return current_mps


def calculate_delta_odometry(deltaMetersTraveled,currentAngle):
    """
    Better for us to just track deltas in our pose rather than try to build the tf here. It will be easier to identify
    trends in what particular maneuvers will make these readings more or less reliable from higher up in the control law.

    :param deltaMetersTraveled: change in how far each wheel has traveled since the last time we went through
    :param currentAngle: current angle of each of the planetary motors
    :return: should return change in x,y position (meters) and orientation(degrees) from a 0,0 starting position
    """
    global steering_state
    global compass_home
    global compass
    print 'compass',compass
    imu_angle=to180(compass['heading'])
    print "imu_angle: ",imu_angle

    avgDeltaMetersTraveled = (deltaMetersTraveled[0] + deltaMetersTraveled[1] + deltaMetersTraveled[2] +
                              deltaMetersTraveled[3]) / 4

    VelocityVector = (currentAngle[0] + currentAngle[1] + currentAngle[2] + currentAngle[3]) / 4

    #normalize tranlsation vector
    VelocityVector+=imu_angle
    print "Velocity_Vector, ",VelocityVector
    imu_frame_VelocityVector=to180(VelocityVector)
    xtranslationComponent = math.sin(np.deg2rad(imu_frame_VelocityVector)) * avgDeltaMetersTraveled
    ytranslationComponent = math.cos(np.deg2rad(imu_frame_VelocityVector)) * avgDeltaMetersTraveled

    return xtranslationComponent,ytranslationComponent

def vector_callback(vector):
    """
    This just updates a global rather than returning incase we want our tf broadcaster to have a different
    Hz resolution from our nav algorithm writing to the teensy
    :param vector: 3 vectors being written from teensy. [speed,velocity_vector,theta_dot]
    :return: nothing
    """
    global vectors
    vectors = {
        'speed': vector.x,  # m/s
        'velocity_vector': vector.y,  # translation
        'theta_dot': vector.z  # rate of rotation
    }

def compass_callback(heading):
    """
    stores arduino compass data in a global dict
    :param data: Vector3
    :return: sets global compass (dict)
    """
    global compass
    compass = {
        'heading': heading.data
    }



def write_to_Teensy(speed,velocity_vector,theta_dot,angles_only):
    """
    this runs everytime the ROS subscriber is updated.Between 15 and 30 hz should be sufficient This writes data to the teensy, and tracks
    for over wrapping on the wheels to avoid twisted wires.
    :param data: list of 3 vectors received from subscriber
    :return: nothing
    """
    # specify behavior in the envent of a point turn
    pointTurnAngle0 = -45
    pointTurnAngle1 = 45
    pointTurnAngle2 = -45
    pointTurnAngle3 = 45
    pointTurnDir0 = -1
    pointTurnDir1 = -1
    pointTurnDir2 = 1
    pointTurnDir3 = 1

    # write to teensy
    if angles_only == False:
        ser.write(str('W0' + calcWheel(speed, velocity_vector, theta_dot, psi0, pointTurnAngle0, pointTurnDir0,angles_only) + '\n'))
        ser.write(str('W1' + calcWheel(speed, velocity_vector, theta_dot, psi1, pointTurnAngle1, pointTurnDir1,angles_only) + '\n'))
        ser.write(str('W2' + calcWheel(speed, velocity_vector, theta_dot, psi2, pointTurnAngle2, pointTurnDir2,angles_only) + '\n'))
        ser.write(str('W3' + calcWheel(speed, velocity_vector, theta_dot, psi3, pointTurnAngle3, pointTurnDir3,angles_only) + '\n'))
    else:
        ser.write(str('W0' + calcWheel(speed, velocity_vector, theta_dot, psi0, pointTurnAngle0, pointTurnDir0,angles_only) + '\n'))
        ser.write(str('W1' + calcWheel(speed, velocity_vector, theta_dot, psi1, pointTurnAngle1, pointTurnDir1,angles_only) + '\n'))
        ser.write(str('W2' + calcWheel(speed, velocity_vector, theta_dot, psi2, pointTurnAngle2, pointTurnDir2,angles_only) + '\n'))
        ser.write(str('W3' + calcWheel(speed, velocity_vector, theta_dot, psi3, pointTurnAngle3, pointTurnDir3,angles_only) + '\n'))
    ser.reset_input_buffer()

def box_car_filter(train,filteredRPM, boxSize):
    """
    boxCarFilter moves a "boxcar" along the data train averaging the last few numbers.Note that his will slightly offset
    the filtered rpm relative to the time sampled. It is correlated with boxSize, since it averages the data seen in the past
    :param train: queue data type only remembers past boxSize
    :return: smoothed_data
    """
    #make it a list
    nextBox = []
    for i in range(0, 4):
        nextBox.append(filteredRPM[i])
    train.insert(0,nextBox)
    if (len(train) > boxSize):  # train should be 3 cars long
        train.pop()
    print "train: ", train
    smoothedRPM = {
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }

    for wheel in range(0,4):
        sum = 0
        boxes = 0#keeps track of which boxes are null
        for box in range(0,boxSize):
            try:
                sum+=train[box][wheel]
                boxes+=1
            except:#its a null
                pass
        try:
            smoothedRPM[wheel] = sum/boxes
        except:
            smoothedRPM[wheel] = 'null'
        boxes = 0
    return smoothedRPM,train

def best_delta_RPM(current_RPM,lastGoodRPM,maxDelta,lastGoodRPMcount,maxNull):
    """
    returns what the modified rpm. not the most recent but the delta from what was perceived to be the last"Good"rpm
    supposed to eliminate high spikes while keeping low drops from high spikes back to the actual value. (abrupt negative deltas
    are to be expected, but never a large positive delta) If more than maxNull values are reported 'null' then we assume
    that the rare case of a large true positive delta has occurred and we reset the lastGoodRPMCount to 0 and set lastGoodRPM
    to the currentRPM
    :param current_RPM:
    :param lastGoodRPM:
    :param maxDelta:
    :param lastGoodRPMcount: list cause its the only one we need to mutate.
    :return:
    """
    filteredRPM = {
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }

    for wheel in range(0,4):#we only care about small going to large not large going to small
        if ((current_RPM[wheel]-lastGoodRPM[wheel]) > maxDelta)and (lastGoodRPMcount[wheel] < maxNull):#spike so report null rpm change
            filteredRPM[wheel]='null'
            lastGoodRPMcount[wheel]+=1
        else:#no spike so record true delta and shift our lastGoodRPM
            filteredRPM[wheel] = current_RPM[wheel]
            lastGoodRPM[wheel] = current_RPM[wheel]
            lastGoodRPMcount[wheel] = 0

    return filteredRPM,lastGoodRPM,lastGoodRPMcount

def idle_noise_filter(currentRPM,desiredRPM):
    """
    This filter should remove the noise that the teensy's wheels sometimes report when we are stationary and not writing a speed
    :param currentRPM: dict
    :param desiredRPM: dict
    :return: idleFilteredRPM dict
    """
    idleFilteredRPM = {
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }
    for i in range(0,4):
        if (currentRPM[i] > 0 and desiredRPM[i] == 0) or steering_state["point_turn"]==True:#we are moving but want to be stationary. assume this is idle noise
            idleFilteredRPM[i] = 0
        else:
            idleFilteredRPM[i] = currentRPM[i]

    return idleFilteredRPM

def propagation_filter():
    """
    This filter
    :return:
    """

def start():
    """
    main method where ROS lives.
    Writes+logs data in the subscriber callback at the frequency "vectors" topic is published
    Odometry+translationVector publisher runs on its own timer publisherHz
    :return: nothing
    """
    #logging
    global count
    global loggingMatrix
    global compass_home


    start = time.time()

    #ROS inits
    rospy.Subscriber("input_vectors", Vector3, vector_callback)
    rospy.Subscriber(topics.ORIENTATION, Int32,compass_callback)
    # rospy.Subscriber("map_orientation",Vector3,map_init_orientation_callback)
    rospy.init_node('TeensyWrite_node')
    rate = rospy.Rate(hz=Hz)
    br = tf.TransformBroadcaster()

    #MAIN LOOP INITS
    totalX = 0
    totalY = 0
    totalMetersTraveled=0
    lastMetersTraveled = [0,0,0,0]
    metersTraveled = {
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }
    lastGoodRPMcount = [0,0,0,0]
    lastGoodRPM={
        0: float(0),
        1: float(0),
        2: float(0),
        3: float(0)
    }
    loopCount = 0

    # LOGGING INITS
    matrix.append(["rawRPM0", "rawRPM1", "rawRPM2", "rawRPM3",
                 "rawAngle0", "rawAngle1", "rawAngle2", "rawAngle3",
                 "desiredRPM0", "desireRPM1", "desiredRPM2", "desiredRPM3",
                 "desiredAngle0", "desiredAngle1", "desiredAngle2", "desiredAngle3",
                 "deltaAngle0", "deltaAngle1", "deltaAngle2", "deltaAngle3",
                 "deltaRPM0", "deltaRPM1", "deltaRPM2", "deltaRPM3",
                 "HighDeltaFilterRPM0", "HighDeltaFilterRPM1", "HighDeltaFilterRPM2", "HighDeltaFilterRPM3",
                 "boxFilterRPM0", "boxFilterRPM1", "boxFilterRPM2", "boxFilterRPM3",
                 "TeensyTimeMicroSeconds"])

    lastTrain = []

    try:
        compass_calibrated = True
        compass_home = compass['heading']#we are assuming that the very first reading we get from PIXHAWK is good
    except:
        rospy.loginfo("no compass data published")
    while not rospy.is_shutdown():
        try:
            #READ/WRITE data to Teensy
            desired_angle, currentAngle, current_RPM, desired_RPM, speedCheck, deltaTics,deltaAngle,deltaHubRPM,teensyTime = getLatestData()
            write_to_Teensy(vectors['speed'],vectors['velocity_vector'],vectors['theta_dot'],angles_only=False)
            #write_to_Teensy(vectors['speed'],vectors['velocity_vector'],vectors['theta_dot'])

            # write_to_Teensy(.7,0,0)

            # for i in metersTraveled.items():
            #     print "MetersTraveled", i
            # for i in desired_angle.items():
            #     print "desired_angle", i
            # for i in currentAngle.items():
            #     print "currentAngle", i
            # for i in current_RPM.items():
            #     print "currentRPM", i
            # for i in desired_RPM.items():
            #     print "desiredRPM", i
            # for i in speedCheck.items():
            #     print "speedCheck", i
            # for i in deltaTics.items():
            #     print "deltaTics", i
            # for i in deltaAngle.items():
            #     print "deltaAngle", i
            # for i in deltaHubRPM.items():
            #     print "deltaHubRPM", i

            #FILTER the RPM noise caused by planetary motors
            maxDelta=20
            if steering_state['translation'] == True:
                maxNull = 6
            else:
                maxNull = 6

            #idleFilteredRPM = idle_noise_filter(current_RPM,desired_RPM)
            filtered_RPM,lastGoodRPM,lastGoodRPMcount = best_delta_RPM(current_RPM,lastGoodRPM,maxDelta,lastGoodRPMcount,maxNull)
            smoothed_RPM,train = box_car_filter(lastTrain,filtered_RPM,boxSize=6)
            idleFilteredRPM = idle_noise_filter(smoothed_RPM,desired_RPM)


            print "idle_filtered",idleFilteredRPM
            #ODOMETRY from filter
            current_MPS=rpm_to_mps(idleFilteredRPM)
            #NOTE: for some reason when we set lastMetersTraveled as a dict it will be mutated in the below for each loop
            #however, setting lastMetersTraveled to a list (as it is currently implemented) fixes that. Unsure as to why this
            #fixes the problem or what exactly the problem is. Just remember that metersTraveled is dict (tuples) and
            #lastMetersTraveled is a list[]
            for i in range(0,4):
                metersTraveled[i]+=(current_MPS[i]/Hz)
            deltaMetersTraveled = {
                0: metersTraveled[0] - lastMetersTraveled[0],
                1: metersTraveled[1] - lastMetersTraveled[1],
                2: metersTraveled[2] - lastMetersTraveled[2],
                3: metersTraveled[3] - lastMetersTraveled[3]
            }
            global compass_home
            print "compass_home: ",compass_home
            print 'compass: ',compass['heading']

            xChange,yChange = 0,0
            xChange, yChange = calculate_delta_odometry(deltaMetersTraveled, currentAngle)


            for i in range(0,4):
                lastMetersTraveled[i]=metersTraveled[i]

            totalX += xChange
            totalY += yChange
            totalMetersTraveled += (deltaMetersTraveled[0] + deltaMetersTraveled[1] + deltaMetersTraveled[2] +
                                    deltaMetersTraveled[3]) / 4

            #flip x and y
            fTotalX = totalX
            fTotalY = totalY
            print "fTotalX: ",fTotalX
            print "fTotalY: ",fTotalY
            #TF broadcaster



            # if compass['heading'] !=0 and compass_calibrated==False:#first pass, relies on 0 init
            #     compass_calibrated = True
            #     compass_home = compass['heading']
            if compass_calibrated == True:
                br.sendTransform((fTotalY, fTotalX, 0),
                                 # tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(compass['heading'])),
                                 (0, 0, 0, 1),
                                 rospy.Time.now(),
                                 topics.ODOMETRY_FRAME,
                                 topics.WORLD_FRAME)
            else:
                rospy.loginfo('Compass warming up...')


            # if compass['calibration_status'] == 3.0:
            #     if compass_calibrated == False:
            #         if map_calibration_status['toggle_init']==1:
            #             compass_calibrated = True
            #
            #             compass_home = compass['orientation']
            #         else:
            #             print "Waiting for [1,0,0] on map_calibration_status topic"
            #     if compass['is_alive'] == 1:
            #         br.sendTransform((fTotalY, fTotalX, 0),
            #                          tf.transformations.quaternion_from_euler(int(compass['orientation']), 0, 0),
            #                          # (0,0,0,1),
            #                          rospy.Time.now(),
            #                          topics.ODOMETRY_FRAME,
            #                          topics.WORLD_FRAME)
            #     else:
            #         print "compass warming up"
            #         br.sendTransform((fTotalY, fTotalX, 0),
            #                          tf.transformations.quaternion_from_euler(0, 0, 0),
            #                          # (0,0,0,1),
            #                          rospy.Time.now(),
            #                          topics.ODOMETRY_FRAME,
            #                          topics.WORLD_FRAME)
            # else:
            #     print "compass not calibrated"
            #     br.sendTransform((fTotalY, fTotalX, 0),
            #                      tf.transformations.quaternion_from_euler(0, 0, 0),
            #                      # (0,0,0,1),
            #                      rospy.Time.now(),
            #                      topics.ODOMETRY_FRAME,
            #                      topics.WORLD_FRAME)

            #LOGGING
            # dataToLog=[]
            # for i in range(0,4):
            #     dataToLog.append(current_RPM[i])
            # for i in range(0,4):
            #     dataToLog.append(currentAngle[i])
            # for i in range(0,4):
            #     dataToLog.append(desired_RPM[i]*-1)
            # for i in range(0,4):
            #     dataToLog.append(desired_angle[i])
            # # for i in range(0,4):
            # #     dataToLog.append(deltaTics[i])
            # for i in range(0,4):
            #     dataToLog.append(deltaAngle[i])
            # for i in range(0,4):
            #     dataToLog.append(deltaHubRPM[i])
            # for i in range(0,4):
            #     dataToLog.append(filtered_RPM[i])
            # for i in range(0,4):
            #     dataToLog.append(smoothed_RPM[i])
            # # for i in range(0,4):
            # #     dataToLog.append(lastGoodRPM[i])
            # # for i in range(0,4):
            # #     dataToLog.append(lastGoodRPMcount[i])
            # dataToLog.append(teensyTime)
            #
            # start_offset =40
            # test_speed = 1.4
            # test_translation=90
            # test_rotation_rate = 0
            #
            # if loopCount > 1:
            #     write_to_Teensy(test_speed,0,0)
            #     if loopCount > 45+start_offset:
            #         write_to_Teensy(test_speed,test_translation,test_rotation_rate)
            #         if loopCount > 120+start_offset:
            #             write_to_Teensy(test_speed,0,0)
            # writer = csv.writer(open('translation_90.csv', "wb"))
            # matrix.append(dataToLog)
            #
            #
            # if loopCount > 160+start_offset:
            #     for entries in matrix:
            #         print matrix
            #         cols = zip(entries)
            #         writer.writerow(cols)
            #     sys.exit()#kills session
        except Exception as e:
            pass
            #print "no data from teensy %s" %e
        rate.sleep()
        loopCount+=1
    rospy.spin()

if __name__ == '__main__':

    global vectors #input vectors
    vectors = {
        'speed': 0, #m/s
        'velocity_vector': 0,  # translation
        'theta_dot': 0  # rate of rotation
    }
    global steering_state #steering state according to the input vectors
    steering_state = {
        "translation": False,
        "regular_turn": False,
        "translation_and_rotation": False,
        "point_turn": False,
        "no_turn": True
    }
    global compass #data from pixhawk
    compass = {
        'heading': 0
    }



    #logging
    matrix = []
    count = 1

    #filters
    train = []

    start()
