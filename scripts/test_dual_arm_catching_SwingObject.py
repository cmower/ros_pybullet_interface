#!/usr/bin/env python3
import sys
import os
import math
import time
import tf2_ros
import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from ros_pybullet_interface.utils import ROOT_DIR
import set_object_state_client


# --- import external library
from py_pack import yaml, np, LA, R

# --- import Hybrid Trajectory Optimization class
from py_pack import hybridto_so
from py_pack import hybridto_dac_so as HybridTO

# ROS message types
from nav_msgs.msg import Odometry

NEW_TRAJ_Yang_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_Yin_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic

WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/robot/end_effector_sponge' # listens for end-effector poses on this topic
ROBOT_BASE_ID = "ros_pybullet_interface/robot/robot_base" # listen for the pose of the robot base


OBJECT_VELOCITY_TOPIC_NAME = "target/filtered_twist"
OBJECT_POSITION_TOPIC_NAME = "target/filtered_map"

def rearrageSolution(initIndex, timePre, posBodyPre, velBodyPre, time, posBody, velBody, posLimb1, velLimb1, posLimb2, velLimb2):
    timeFree = np.reshape(timePre[0, 0:initIndex], (1, len(timePre[0, 0:initIndex])))
    posBodyFree = posBodyPre[:, 0:initIndex]
    velBodyFree = velBodyPre[:, 0:initIndex]
    row, column = posBodyPre.shape
    posLimb1Free = np.zeros((row + 1, initIndex))
    posLimb1Free = np.repeat(posLimb1[:, 0], initIndex, axis=1)
    posLimb2Free = np.zeros((row + 1, initIndex))
    posLimb2Free = np.repeat(posLimb2[:, 0], initIndex, axis=1)
    velLimb1Free = np.zeros((row, initIndex))
    velLimb2Free = np.zeros((row, initIndex))
    forLimb1Free = np.zeros((row, initIndex))
    forLimb2Free = np.zeros((row, initIndex))

    timeArray = np.hstack((timeFree, time[0, 1:] + timeFree[0, -1]))
    posBodyArray = np.hstack((posBodyFree, posBody[:, 1:]))
    velBodyArray = np.hstack((velBodyFree, velBody[:, 1:]))
    posLimb1Array = np.hstack((posLimb1Free, posLimb1[:, 1:]))
    velLimb1Array = np.hstack((velLimb1Free, velLimb1[:, 1:]))
    # forLimb1Array = np.hstack((forLimb1Free, forLimb1))
    posLimb2Array = np.hstack((posLimb2Free, posLimb2[:, 1:]))
    velLimb2Array = np.hstack((velLimb2Free, velLimb2[:, 1:]))
    # forLimb2Array = np.hstack((forLimb2Free, forLimb2))

    trajObjPlan = np.vstack((np.vstack((timeArray, posBodyArray)), velBodyArray))
    trajYangPlan = np.vstack((np.vstack((timeArray, posLimb1Array)), velLimb1Array))
    trajYinPlan = np.vstack((np.vstack((timeArray, posLimb2Array)), velLimb2Array))

    return trajObjPlan, trajYangPlan, trajYinPlan

class PlanInterpWithTO:

    def __init__(self, initBaseYang, initBaseYin, initEEYang, initEEYin, robot1_name, robot2_name, \
                 paramFilePath = 'configs/config_swing.yaml', initFilePath = 'data/init_trajopt_swing.npy'):

        # Name of node
        self.name = rospy.get_name()

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)
        self.trajYangPlan = np.empty(0)
        self.trajYinPlan = np.empty(0)

        # start punlishers
        self.new_Yangtraj_publisher = rospy.Publisher(f"{robot1_name}/{NEW_TRAJ_Yang_TOPIC}", Float64MultiArray, queue_size=1)
        self.new_Yintraj_publisher = rospy.Publisher(f"{robot2_name}/{NEW_TRAJ_Yin_TOPIC}", Float64MultiArray, queue_size=1)
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        # get the path to this catkin ws
        self.initFile = os.path.join(ROOT_DIR, initFilePath)
        paramFile = os.path.join(ROOT_DIR, paramFilePath)
        with open(paramFile, 'r') as ymlfile:
            params = yaml.load(ymlfile, Loader=yaml.SafeLoader)


        self.uncertainty = np.array(params['estimation']['uncertainty'])

        self.finObjPos = np.array(params['object']['finObjPos'])
        self.maxObjPos = np.array(params['object']['maxObjPos'])
        self.finObjVel = np.array(params['object']['finObjVel'])
        self.maxObjVel = np.array(params['object']['maxObjVel'])

        self.slackObjPos = np.array(params['object']['slackObjPos'])
        self.slackObjVel = np.array(params['object']['slackObjVel'])

        self.initArmEEPos1 = initEEYang
        self.initArmEEPos2 = initEEYin

        self.minArmEEPos1 = np.array(params['robotYang']['minWorkspace']) + initBaseYang
        self.maxArmEEPos1 = np.array(params['robotYang']['maxWorkspace']) + initBaseYang

        self.minArmEEPos2 = np.array(params['robotYin']['minWorkspace']) + initBaseYin
        self.maxArmEEPos2 = np.array(params['robotYin']['maxWorkspace']) + initBaseYin


    def publishRobotTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajRobotPlan)

        if message != None:
            self.new_Robottraj_publisher.publish(message)


    def publishYangTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajYangPlan)

        if message != None:
            self.new_Yangtraj_publisher.publish(message)

    def publishYinTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajYinPlan)

        if message != None:
            self.new_Yintraj_publisher.publish(message)

    def publishObjTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajObjPlan)

        if message != None:
            self.new_Objtraj_publisher.publish(message)


    def np2DtoROSmsg(self, data2Darray):

        # check if there is nothing to publish
        if data2Darray.size == 0:
            return None

        r, c = data2Darray.shape

        # info: http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
        # Pack trajectory msg
        msg = Float64MultiArray()

        # specify that the array has 2 dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        # info for reconstruction of the 2D array
        msg.layout.dim[0].label  = "rows"
        msg.layout.dim[0].size   = r
        msg.layout.dim[1].label  = "columns"
        msg.layout.dim[1].size   = c

        # add data as flattened numpy array
        msg.data = data2Darray.flatten('C') # row major flattening


        # time is missing from this message
        # msg.header.stamp = rospy.Time.now()
        return msg


    def buildTO(self):

        # create instance of the Hybrid optimisation class
        self.HybOpt_DAC = HybridTO.HybridTOClass_DAC(paramFile="/../config/paramSwingObject.yml")
        # build the problem
        self.HybProb = self.HybOpt_DAC.buildProblem()
        # build the problem with warm start
        self.HybProb_warmstart = self.HybOpt_DAC.buildProblem(warmStart=True)

        # get initial guess
        self.xInit = self.HybOpt_DAC.buildInitialGuess()
        with open(self.initFile, 'rb') as f:
            self.xInit = np.load(f)


    def solveTO(self, initObjPos, initObjVel, normVector, cntPntVector, initEEAttYang_Quat, initEEAttYin_Quat):

        lbx, ubx, lbg, ubg, cf, gf = self.HybOpt_DAC.buildBounds(initObjPos, self.finObjPos, self.maxObjPos, self.slackObjPos,
                                                          initObjVel,  self.finObjVel,  self.maxObjVel,  self.slackObjVel,
                                                          self.initArmEEPos1, self.minArmEEPos1, self.maxArmEEPos1,
                                                          self.initArmEEPos2, self.minArmEEPos2, self.maxArmEEPos2, normVector)

        solFlag, xSolution = self.HybOpt_DAC.solveProblem(self.HybProb, self.xInit, lbx, ubx, lbg, ubg, cf, gf, normVector, cntPntVector)
        with open(self.initFile, 'wb') as f:
            np.save(f, np.array(xSolution))
        # solFlag, xSolution = self.HybOpt_DAC.solveProblem(self.HybProb_warmstart, self.xInit, lbx, ubx, lbg, ubg, cf, gf, normVector)

        # decode solution
        if (solFlag):
            timeArray, posBodyArray, velBodyArray, posLimb1Array, velLimb1Array, forLimb1Array,\
            posLimb2Array, velLimb2Array, forLimb2Array, alphaArray = self.HybOpt_DAC.decodeSol(xSolution, normVector)

            # self.HybOpt_DAC.plotResult(timeArray, posBodyArray, velBodyArray, posLimb1Array, velLimb1Array,
            #                            forLimb1Array, posLimb2Array, velLimb2Array, forLimb2Array, animateFlag=False)
            m, n = posLimb1Array.shape
            # for i in range(n):
            #     posLimb1Array[3:, i] = endAttYang_Quat
            #     posLimb2Array[3:, i] = endAttYin_Quat
            np_initEEAttYang_Quat = np.array(initEEAttYang_Quat)
            posLimb1Array[3:7, 0:2] = np.vstack((np_initEEAttYang_Quat, np_initEEAttYang_Quat)).T
            np_initEEAttYin_Quat = np.array(initEEAttYin_Quat)
            posLimb2Array[3:7, 0:2] = np.vstack((np_initEEAttYin_Quat,np_initEEAttYin_Quat)).T
            # posLimb1Array[3:6, 0] = endAttYang
            # posLimb2Array[3:6, 0] = endAttYin


        return solFlag, timeArray, posBodyArray, velBodyArray, posLimb1Array,velLimb1Array, forLimb1Array, posLimb2Array, velLimb2Array, forLimb2Array


    def stateMachine(self, posInitTraj):

        while (True):

            pose, velocity = Initials.getPosVel()

            # compute Euclidean distance, to see if object is in the expected location
            dist = LA.norm(pose[0:3]-posInitTraj[0:3])
            if dist <= self.uncertainty:
                commandFlag = True
                break

        return commandFlag




class PredictionWithTO():
    def __init__(self, initBaseYang, initBaseYin, paramFilePath = 'configs/config_swing.yaml',
                 initFilePath = 'data/init_prediction_swing.npy'):
        # Name of node
        self.name = rospy.get_name()

        # get the path to this catkin ws
        self.initFile = os.path.join(ROOT_DIR, initFilePath)
        paramFile = os.path.join(ROOT_DIR, paramFilePath)
        with open(paramFile, 'r') as ymlfile:
            params = yaml.load(ymlfile, Loader=yaml.SafeLoader)

        self.finObjPos = np.array(params['object']['finObjPos'])
        self.maxObjPos = np.array(params['object']['maxObjPos'])
        self.finObjVel = np.array(params['object']['finObjVel'])
        self.maxObjVel = np.array(params['object']['maxObjVel'])

        self.slackObjPos = np.array(params['object']['slackObjPos'])
        self.slackObjVel = np.array(params['object']['slackObjVel'])
        self.reducedNode = int(params['object']['reducedNode'])

        # simplified workspace of robot: r; installation position of left robot and right robot: L0, R0
        self.r = float(params['robotYang']['workspace'])
        self.L0 = initBaseYang
        self.R0 = initBaseYin


    def buildTO(self):

        #  create instance of the Hybrid optimisation class
        self.HybOpt3D_SRB = hybridto_so.HybridTOClass3D(paramFile="/../config/paramSwingObject.yml")

        # build the problem
        self.HybProb = self.HybOpt3D_SRB.buildProblem()
        # build the problem with warm start
        self.HybProb_warmstart = self.HybOpt3D_SRB.buildProblem(warmStart=True)

        # initialize the optimization
        self.xInit_SRB = self.HybOpt3D_SRB.buildInitialGuess()
        with open(self.initFile, 'rb') as f:
            self.xInit_SRB = np.load(f)


    def solveTO(self, initObjPos, initObjVel):

        lbx, ubx, lbg, ubg, cf, gf = self.HybOpt3D_SRB.buildBounds(initObjPos, self.finObjPos, self.maxObjPos, 0.0,
                                                              initObjVel, self.finObjVel, self.maxObjVel, 0.0)

        start_time = time.time()

        # solve problem
        preFlag, xSolution = self.HybOpt3D_SRB.solveProblem(self.HybProb, self.xInit_SRB, lbx, ubx, lbg, ubg, cf, gf)
        with open(self.initFile, 'wb') as f:
            np.save(f, np.array(xSolution))
        # preFlag, xSolution = self.HybOpt3D_SRB.solveProblem(self.HybProb_warmstart, self.xInit_SRB, lbx, ubx, lbg, ubg, cf, gf)

        # decode solution
        if (preFlag):
            timePre, posBodyPre, velBodyPre, force = self.HybOpt3D_SRB.decodeSol(xSolution, animateFlag=False)

            start_time_contactSelection = time.time()
            initIndex, normIndex, normVector, cntPntVector = self.HybOpt3D_SRB.contactSelection(posBodyPre, self.r, self.L0, self.R0)
            print("#---Contact selection time: %s seconds ---#" % (time.time() - start_time_contactSelection))
            print("#---Prediction time: %s seconds ---#" % (time.time() - start_time))
            print("#---Index for the initial state for TO: %s ---#" % (initIndex))
            print("#---Normal vector of contact surface for TO: %s ---#" % (normVector))

            #  clamp reduce index
            reduce = max(0, min(self.reducedNode, initIndex))
            initIndex = initIndex-reduce

            return initIndex, normVector, cntPntVector, timePre, posBodyPre, velBodyPre


class InitialsOfPrediciton():

    def __init__(self, paramFilePath = 'configs/config_flying.yaml'):

        # Name of node
        self.name = rospy.get_name()

        self.state_listen_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.state_listen_buff)

        # Subscribe target object estimated callback
        rospy.Subscriber(OBJECT_POSITION_TOPIC_NAME, Odometry, self.readPose)
        rospy.Subscriber(OBJECT_VELOCITY_TOPIC_NAME, Odometry, self.readVelocity)

        self.objectState = np.zeros(13)

        # get the path to this catkin ws
        paramFile = os.path.join(ROOT_DIR, paramFilePath)
        with open(paramFile, 'r') as ymlfile:
            params = yaml.load(ymlfile, Loader=yaml.SafeLoader)

        self.condition = float(params['estimation']['condition'])


    def readPose(self, msg):

        # geometry_msgs/PoseWithCovarianceStamped
        # pose
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        orientation = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        covariance = msg.pose.covariance

        self.objectState[:3] = position
        self.objectState[3:7] = orientation


    def readVelocity(self, msg):

        # geometry_msgs/PoseWithCovarianceStamped

        # twist
        linear_velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        angular_velocity = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        covariance = msg.twist.covariance

        self.objectState[7:10] = linear_velocity
        self.objectState[10:13] = angular_velocity

    def getPosVel(self):

        ROT = R.from_quat(self.objectState[3:7])
        eulerAngle = ROT.as_euler('ZYX')
        pose = np.hstack((self.objectState[0:3], eulerAngle))

        velocity = self.objectState[7:13]
        #  transform local velocity to gloabal frame
        global_lin_vel = ROT.as_matrix().dot(velocity[0:3])
        global_ang_vel = ROT.as_matrix().dot(velocity[3:])

        global_velocity = np.hstack((global_lin_vel.T, global_ang_vel.T))

        return pose, global_velocity

    def readInitials(self, robot_name):

        rospy.loginfo(f"{self.name}: Reading for /tf topic the position and orientation of the robot")
        # Read the position and orientation of the robot from the /tf topic
        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.state_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{ROBOT_BASE_ID}",
                                                             rospy.Time())
                break
            except:
                rospy.logwarn(f"{self.name}: /tf topic does NOT have {robot_name}/{ROBOT_BASE_ID}")

        base_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                            trans.transform.rotation.w]
        base_orient_euler = R.from_quat(base_orient_quat).as_euler('ZYX')

        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.state_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{END_EFFECTOR_FRAME_ID}",
                                                             rospy.Time())
                break
            except:
                rospy.logwarn(f"{self.name}: /tf topic does NOT have {robot_name}/{END_EFFECTOR_FRAME_ID}")

        end_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        end_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                            trans.transform.rotation.w]
        end_orient_euler = R.from_quat(end_orient_quat).as_euler('ZYX')

        return base_position, base_orient_euler, base_orient_quat, end_position, end_orient_euler, end_orient_quat

    def startEstimation(self):

        while(True):

            pose, velocity= self.getPosVel()

            # return the estimation results if the object pass the predefined location
            if pose[1] >= self.condition:
                break

        return pose, velocity



if __name__=='__main__':

    rospy.sleep(1.0)

    # --- setup the ros interface --- #
    rospy.init_node('test_dual_arm_capturing_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")

    # check if the name of the robot is provided
    if rospy.has_param('~robot1_name') and rospy.has_param('~robot2_name'):
        robot1_name = rospy.get_param('~robot1_name')
        robot2_name = rospy.get_param('~robot2_name')
    else:
        rospy.logerr(f"The name of the robots is not set in {rospy.get_name()}")
        sys.exit(0)


    freq = 100

    Initials = InitialsOfPrediciton()

    # get the initial position and orientation of the robot
    basePosYang, baseAttYang, baseAttYang_Quat, endPosYang, endAttYang, endAttYang_Quat = Initials.readInitials("yang")
    basePosYin, baseAttYin, baseAttYin_Quat, endPosYin, endAttYin, endAttYin_Quat = Initials.readInitials("yin")
    print(basePosYang, basePosYin, endPosYang, endPosYin)

    PredictionWithTO = PredictionWithTO(basePosYang, basePosYin)
    PlanInterpWithTO = PlanInterpWithTO(basePosYang, basePosYin, endPosYang, endPosYin, robot1_name, robot2_name)
    rospy.loginfo("%s: node started.", PlanInterpWithTO.name)

    ''' ATTENTION: replace the following with parameter from ROS as:
                # Get ros parameters
                only_obj = rospy.get_param('~only_object')
            '''
    path2extrPck = os.environ['PATH2HYBRIDMPC']
    paramFile = os.path.join(path2extrPck, "py_pack/config/paramSwingObject.yml")
    with open(paramFile, 'r') as ymlfile:
        params = yaml.load(ymlfile, Loader=yaml.SafeLoader)

    ori_representation = params['TOproblem']['ori_representation']
    objLength = params['object']['shape']['rectangle']['length']

    hangPoint = np.array(params['hangPoint'])
    ropeLength = params['ropeLength']
    startAngle = params['startAngle']

    # build the prediction problem
    PredictionWithTO.buildTO()
    # build the TO problem
    PlanInterpWithTO.buildTO()

    #---SET OBJECT STATE FOR INITIALIZATION: Moving, Swinging, Flying---#
    objectState = "Swing"

    if objectState == "Moving":
        # get bounds of variables and constraints
        if ori_representation == "euler":
            # euler representation initialization #
            initObjPos = np.array([0.0, -2.5, 0.3, -90.0 / 180 * np.pi, 0, 0])
            #  real
            # initObjPos = np.array([0.188, 1.06, 0.90, 100 / 180 * np.pi, 0, 0])

            pos = initObjPos[0:3]
            quat = R.from_euler('ZYX', initObjPos[3:6]).as_quat()
        elif ori_representation == "quaternion":
            # quaternion representation initialization #
            initObjPos = np.array([0, 0, 0.5, 0, 0, 0, 1])
            pos = initObjPos[0:3]
            quat = initObjPos[3:7]

        initObjVel = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        #  real
        # initObjVel = np.array([0.0, -0.3, 0.0, 0.0, 0.0, 0.0])
        lin_vel = initObjVel[0:3];        ang_vel = initObjVel[3:6]

    if objectState == "Flying":
        if ori_representation == "euler":
            # euler representation initialization #
            initObjPos = np.array([0, -2.0, -1.0, 0, 0, 0])
            pos = initObjPos[0:3]
            quat = R.from_euler('ZYX', initObjPos[3:6]).as_quat()

        initObjVel = np.array([0, 2.5, 5.5, 0.0, 0.0, 0.0])
        lin_vel = initObjVel[0:3];        ang_vel = initObjVel[3:6]

    if objectState == "Swing":
       
        initObjPos = np.array([0, (ropeLength + objLength / 2) * np.sin(startAngle* np.pi / 180.) - 0.0, hangPoint[2]
                               - (ropeLength + objLength / 2) * np.cos(startAngle* np.pi / 180.), 0, 0, startAngle* np.pi / 180.])
        pos = initObjPos[0:3]
        quat = R.from_euler('ZYX', initObjPos[3:6]).as_quat()

        initObjVel = np.array([0, 0, 0, 0.0, 0.0, 0.0])
        lin_vel = initObjVel[0:3];        ang_vel = initObjVel[3:6]

    set_object_state_client.setObjState(pos, quat, lin_vel*0, ang_vel*0, 'target')
    rospy.sleep(2)
    set_object_state_client.setObjState(pos, quat, lin_vel, ang_vel, 'target')

    # get object and robot state from pybullet
    # initObjPos, initObjVel = Initials.startEstimation()
    # initObjVel = np.array([0, 0, 0, 0.0, 0.0, 0.0])
    print("The estimated pose of the object is :", initObjPos)
    print("The estimated velocity of the object is :", initObjVel)

    # initObjVel[0] *= 0.
    # # initObjVel[1] *= -1
    # initObjVel[2:6] *= 0.
    # uncertainty = 1.57-initObjPos[3]
    # if abs(uncertainty) >= np.deg2rad(20.):
    #     print('uncertainty:', np.rad2deg(uncertainty))
    #     exit()
    #
    # if abs(uncertainty) >= np.deg2rad(10.):
    #     print('uncertainty:', np.rad2deg(uncertainty))
    #     uncertainty = max(np.deg2rad(10.), min(uncertainty, np.deg2rad(-10.)))
    #
    #
    # initObjPos[3] = 3.14 - uncertainty

    # if (initObjVel[1] < -0.37 or initObjVel[1] > -0.20):
    #     print('initObjVel:', initObjVel)
    #     exit()
    start_time = time.time()


    # solve the trajectory optimization for prediction
    initIndex, normVector, cntPntVector, timePre, posBodyPre, velBodyPre = PredictionWithTO.solveTO(initObjPos, initObjVel)

    # solve the TO problem
    solFlag, timeSeq, posBody, velBody, posLimb1, velLimb1, forLimb1, posLimb2, velLimb2, forLimb2 = \
        PlanInterpWithTO.solveTO(posBodyPre[:, initIndex], velBodyPre[:, initIndex], normVector, cntPntVector, endAttYang_Quat, endAttYin_Quat)
    print("start optimization:", posBodyPre[:, initIndex])


    # commandFlag = PlanInterpWithTO.stateMachine(posBodyPre[:, initIndex])
    # print('solFlag =', solFlag, 'commandFlag =',commandFlag)
    commandFlag = False

    # Visualize the planning result for capturing swinging object
    PlanInterpWithTO.HybOpt_DAC.plotResult(timeSeq, posBody, velBody, posLimb1, velLimb1, forLimb1, posLimb2, velLimb2, forLimb2, animateFlag=False)


    if commandFlag == True:
        trajObjPlan = np.vstack((np.vstack((timeSeq, posBody)), velBody))
        trajYangPlan = np.vstack((np.vstack((timeSeq, posLimb1)), velLimb1))
        trajYinPlan = np.vstack((np.vstack((timeSeq, posLimb2)), velLimb2))


    else:
        trajObjPlan, trajYangPlan, trajYinPlan = \
            rearrageSolution(initIndex, timePre, posBodyPre, velBodyPre, timeSeq, posBody, velBody, posLimb1, velLimb1, posLimb2, velLimb2)

    PlanInterpWithTO.trajObjPlan = trajObjPlan
    PlanInterpWithTO.trajYangPlan = trajYangPlan
    PlanInterpWithTO.trajYinPlan = trajYinPlan

    print('Computation time:', time.time()-start_time)

    if solFlag == True:
        rospy.loginfo("TO problem solved, publish the state of object and robots!")
        PlanInterpWithTO.writeCallbackTimerYang = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishYangTrajectory)
        PlanInterpWithTO.writeCallbackTimerYin = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishYinTrajectory)
        PlanInterpWithTO.writeCallbackTimerObj = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishObjTrajectory)

        rospy.loginfo("TO problem solved, set visual object state in bullet!")

    rospy.spin()
