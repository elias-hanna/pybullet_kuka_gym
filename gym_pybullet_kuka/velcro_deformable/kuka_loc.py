import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data

def accurateCalculateInverseKinematics(kukaId, endEffectorId, targetPos, targetOrn, threshold, maxIter):
  closeEnough = False
  iter = 0
  dist2 = 1e30
  numJoints = p.getNumJoints(kukaId)
  while (not closeEnough and iter < maxIter):
    jointPoses = p.calculateInverseKinematics(kukaId, endEffectorId, targetPosition=targetPos, targetOrientation=targetOrn)
    jointPoses = list(jointPoses)+[0,0]
    for i in range(numJoints):
      p.resetJointState(kukaId, i, jointPoses[i])
    ls = p.getLinkState(kukaId, endEffectorId)
    newPos = ls[4]
    diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    closeEnough = (dist2 < threshold)
    iter = iter + 1
  #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
  return jointPoses


class Kuka:

  def __init__(self,
               urdfRootPath=pybullet_data.getDataPath(),
               timeStep=0.01,
               useInverseKinematics=True):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.maxVelocity = .35
    self.maxForce = 200
    self.fingerAForce = 100
    self.fingerBForce = 100
    self.fingerTipForce = 100
    self.useInverseKinematics = useInverseKinematics
    self.useSimulation = 1
    self.useNullSpace = 1
    self.useOrientation = 1
    self.kukaEndEffectorIndex = 6
    self.kukaGripperIndex = 7
    #lower limits for null space
    self.ll=[-.967,-2 ,-2.96,0.19,-2.96,-2.09,-3.05]
    #upper limits for null space
    self.ul=[.967,2 ,2.96,2.29,2.96,2.09,3.05]
    #joint ranges for null space
    self.jr=[5.8,4,5.8,4,5.8,4,6]
    #restposes for null space
    self.rp=[0,0,0,0.5*math.pi,0,-math.pi*0.5*0.66,0]
    #joint damping coefficent
    self.jd=[0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001]
    self.reset()

  def reset(self):
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    objects = p.loadSDF(os.path.join(self.urdfRootPath,"kuka_iiwa/kuka_with_gripper2.sdf"))
    # objects = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")
    self.kukaUid = objects[0]
    # for i in range (p.getNumJoints(self.kukaUid)):
     # print(p.getJointInfo(self.kukaUid,i))
    p.resetBasePositionAndOrientation(self.kukaUid,[-0.100000,0.000000,0.070000],[0.000000,0.000000,0.000000,1.000000])

    ### Other Method ###
    self.endEffectorPos = [0,0,0]
    self.endEffectorAngle = math.pi
    # orn = p.getQuaternionFromEuler([math.pi/2,math.pi/2,math.pi/2])
    orn = p.getQuaternionFromEuler([math.pi/2,0,0])
    position = [0.52, 0.21, 0.]
    
    self.jointPositions = accurateCalculateInverseKinematics(self.kukaUid,
                                                             self.kukaEndEffectorIndex,
                                                             position,
                                                             orn,
                                                             0.01,
                                                             1000)
    
    self.jointPositions = list(self.jointPositions)[0:8]+[-0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200]
    
    ### Initial Method ###
    # self.jointPositions=[ 0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048, -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200]
    # self.jointPositions=[ 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 3.14, .299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200]
    
    self.numJoints = p.getNumJoints(self.kukaUid)
    for jointIndex in range (self.numJoints):
      p.resetJointState(self.kukaUid,jointIndex,self.jointPositions[jointIndex])
      p.setJointMotorControl2(self.kukaUid,jointIndex,p.POSITION_CONTROL,targetPosition=self.jointPositions[jointIndex],force=self.maxForce)

    # self.endEffectorPos = [0.537,0.0,0.5]
    self.endEffectorPos = position
    self.endEffectorAngle = 0

    self.motorNames = []
    self.motorIndices = []

    for i in range (self.numJoints):
      jointInfo = p.getJointInfo(self.kukaUid,i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)
    p.stepSimulation()

  def getActionDimension(self):
    if (self.useInverseKinematics):
      return len(self.motorIndices)
    return 6

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    state = p.getLinkState(self.kukaUid,self.kukaGripperIndex)
    pos = state[0]
    orn = state[1]
    euler = p.getEulerFromQuaternion(orn)

    observation.extend(list(pos))
    observation.extend(list(euler))

    return observation

  def closeGripper(self, motorCommands):
    # if (self.useInverseKinematics):
    if (True):

      da = motorCommands[3]
      fingerAngle = motorCommands[4]

      self.endEffectorAngle = self.endEffectorAngle + da

      #fingers
      # p.setJointMotorControl2(self.kukaUid,7,p.POSITION_CONTROL,targetPosition=self.endEffectorAngle,force=self.maxForce)
      p.setJointMotorControl2(self.kukaUid,8,p.POSITION_CONTROL,targetPosition=-fingerAngle,force=self.fingerAForce,targetVelocity=0,maxVelocity=self.maxVelocity, positionGain=0.3,velocityGain=1)
      p.setJointMotorControl2(self.kukaUid,11,p.POSITION_CONTROL,targetPosition=fingerAngle,force=self.fingerBForce,targetVelocity=0,maxVelocity=self.maxVelocity, positionGain=0.3,velocityGain=1)

      p.setJointMotorControl2(self.kukaUid,10,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,13,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)

    else:
      for action in range (len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,motor,p.POSITION_CONTROL,targetPosition=motorCommands[action],force=self.maxForce)
    

  def applyAction(self, motorCommands):

    if (self.useInverseKinematics):
      dx = motorCommands[0]
      dy = motorCommands[1]
      dz = motorCommands[2]
      da = motorCommands[3]
      fingerAngle = motorCommands[4]


      pos=[0,0,0]
      pos[0] = motorCommands[0]
      pos[1] = motorCommands[1]
      pos[2] = motorCommands[2]



      self.endEffectorPos[0] = self.endEffectorPos[0]+dx
      if (self.endEffectorPos[0]>1):
        self.endEffectorPos[0]=1
      if (self.endEffectorPos[0]<0.0):
        self.endEffectorPos[0]=0.0
      self.endEffectorPos[1] = self.endEffectorPos[1]+dy
      if (self.endEffectorPos[1]<-0.17):
        self.endEffectorPos[1]=-0.17
      if (self.endEffectorPos[1]>1):
        self.endEffectorPos[1]=1
      self.endEffectorPos[2] = self.endEffectorPos[2]+dz
      if (self.endEffectorPos[2]<0):
        self.endEffectorPos[2]=0


      self.endEffectorAngle = self.endEffectorAngle + da
      # self.endEffectorAngle = math.pi

      # pos = self.endEffectorPos
      orn = p.getQuaternionFromEuler([0,-math.pi,0]) # -math.pi,yaw])
      if (self.useNullSpace==1):

        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,self.endEffectorPos,orn,self.ll,self.ul,self.jr,self.rp)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,self.endEffectorPos,lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr, restPoses=self.rp)
      else:

        if (self.useOrientation==1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,self.endEffectorPos,orn,jointDamping=self.jd)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,self.kukaEndEffectorIndex,self.endEffectorPos)

      if (self.useSimulation):
        for i in range (self.kukaEndEffectorIndex+1):
          #print(i)
          p.setJointMotorControl2(bodyUniqueId=self.kukaUid,jointIndex=i,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[i],targetVelocity=0,force=self.maxForce,maxVelocity=self.maxVelocity, positionGain=0.3,velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range (self.numJoints):
          p.resetJointState(self.kukaUid,i,jointPoses[i])
      #fingers
      p.setJointMotorControl2(self.kukaUid,7,p.POSITION_CONTROL,targetPosition=self.endEffectorAngle,force=self.maxForce)
      p.setJointMotorControl2(self.kukaUid,8,p.POSITION_CONTROL,targetPosition=-fingerAngle,force=self.fingerAForce,targetVelocity=0,maxVelocity=self.maxVelocity, positionGain=0.3,velocityGain=1)
      p.setJointMotorControl2(self.kukaUid,11,p.POSITION_CONTROL,targetPosition=fingerAngle,force=self.fingerBForce,targetVelocity=0,maxVelocity=self.maxVelocity, positionGain=0.3,velocityGain=1)

      p.setJointMotorControl2(self.kukaUid,10,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,13,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)

    else:
      # for action in range(len(motorCommands)):
      for action in range(6):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,motor,p.POSITION_CONTROL,targetPosition=motorCommands[action],force=self.maxForce)
      
      p.setJointMotorControl2(self.kukaUid,8,p.POSITION_CONTROL,targetPosition=-0.2,force=self.fingerAForce,targetVelocity=0.5,maxVelocity=self.maxVelocity, positionGain=1.3,velocityGain=1)
      p.setJointMotorControl2(self.kukaUid,11,p.POSITION_CONTROL,targetPosition=0.2,force=self.fingerBForce,targetVelocity=0.5,maxVelocity=self.maxVelocity, positionGain=1.3,velocityGain=1)

      p.setJointMotorControl2(self.kukaUid,10,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
      p.setJointMotorControl2(self.kukaUid,13,p.POSITION_CONTROL,targetPosition=0,force=self.fingerTipForce)
