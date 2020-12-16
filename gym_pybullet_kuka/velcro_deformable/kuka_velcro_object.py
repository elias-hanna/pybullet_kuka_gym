from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import os, inspect
from gym import spaces
import time
import pybullet as p
from gym_pybullet_kuka.velcro_deformable.kuka_loc import Kuka
import gym_pybullet_kuka.data as data_set
import numpy as np
import pybullet_data
import distutils.dir_util
import glob
import gym
import uuid
import sys
import math

tmp_dir = os.path.dirname(sys.modules['__main__'].__file__) + "/tmp"
class KukaVelcroObject(KukaGymEnv):
  """
  Class for Kuka environment with an object following a given path
  """

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               isEnableSelfCollision=True,
               renders=False,
               isDiscrete=False,
               maxSteps=30,
               controlWrist=True,
               axes=[[0,1,1]]):
    """Initializes the KukaVelcroObject. 
    Args:
      urdfRoot: The directory from which to load environment URDF's.
      isEnableSelfCollision: If true, enable self-collision.
      renders: If true, render the bullet GUI.
      isDiscrete: If true, the action space is discrete. If False, the
        action space is continuous.
      maxSteps: The maximum number of actions per episode.
      controlWrist: if True, the wrist of the robot is also controlled during learning
    """
    self._controlWrist = controlWrist
    self._isDiscrete = isDiscrete
    self._timeStep = 1./240.
    self._urdfRoot = urdfRoot
    self._isEnableSelfCollision = isEnableSelfCollision
    self._renders = renders
    self.envId = uuid.uuid4()
    self._time = 0
    self._max_time = 10 #seconds
    self._axes=axes
    self._bottomAnchorsConstraintsIds = []
    self._centralAnchorsConstraintsIds = []
    self._upperAnchorsConstraintsIds = []
    self._bottomAxisLength = 0.05
    self._centralAxisLength = 0.05
    self._upperAxisLength = 0.05
    # Define observation and action space
    # self.action_space = spaces.Discrete(8)
    # self.observation_space = spaces.Discrete(1)
    self.observation_space = spaces.Box(low=0, high=np.inf, shape=(1,))
    self.action_space = spaces.Box(low=-np.inf, high=np.inf, shape=(8,))
    if self._renders:
      self.cid = p.connect(p.SHARED_MEMORY)
      if (self.cid<0):
        self.cid = p.connect(p.GUI)
      p.resetDebugVisualizerCamera(1.5,200,-40,[0.52,-0.2,-0.33])
    else:
      self.cid = p.connect(p.DIRECT)
    # self._seed()
    self.viewer = None

  def reset(self):
    """
    Environment reset called at the beginning of an episode.
    """
    self._time = 0

    ## Use RESET_USE_DEFORMABLE_WORLD to be able to spawn deformable objects in sim
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    
    plane = p.loadURDF(os.path.join(self._urdfRoot,"plane.urdf"),[0,0,-1])
    
    table = p.loadURDF(os.path.join(self._urdfRoot,"table/table.urdf"), 0.5000000,0.00000,-.820000,0.000000,0.000000,0.0,1.0)
    # self.planeId = plane[0]
    # self.tableId = table[0]

    p.setGravity(0,0,-10)

    # Set Use Inverse Kinematics to false as we learn in joint space
    self._kuka = Kuka(urdfRootPath=self._urdfRoot,
                      timeStep=self._timeStep,
                      useInverseKinematics=False)

    p.stepSimulation()
    
    ## PARAMETERS ##
    
    radius = 0.03
    boxId = p.createCollisionShape(p.GEOM_BOX,
                                   halfExtents=[radius, radius, radius])
    
    # Link base parameters
    baseOrientation = [0, 0, 0, 1]
    baseMass = 0
    baseVisualShapeId = -1
    linkParentIndices = [0]
    
    # Link parameters
    damping = 0
    
    linkMasses = [0.5]
    linkVisualShapeIndices = [-1]
    linkPositions = [[0, 0, 1]]
    linkOrientations = [[0, 0, 0, 1]]
    linkInertialFramePositions = [[0, 0, 0]]
    linkInertialFrameOrientations = [[0, 0, 0, 1]]
    linkJointTypes = [p.JOINT_PRISMATIC]
    linkCollisionShapeIndices = [boxId]
    
    # ## SET 1, VERTICAL DEFORMABLE OBJECT ##
    # # Link parameters (axes and basePosition)
    # linkJointAxis = [[-1,0, 1]]
    # basePosition = [0.25, -0.5, -1]
    
    # centralLinkJointAxis = [[1,0,0]]
    # centralBasePosition = [0.25, -1, -1]
    
    # upperLinkJointAxis = [[1,0,1]]
    # upperBasePosition = [0.25, -1.5, -1]
    
    # # Deformable object parameters
    # basePos=[0,0,-1.85]
    # baseOr = [1, 0, 1, 0]
    
    ## SET 2, HORIZONTAL DEFORMABLE OBJECT ##
    # Link parameters (axes and basePosition)
    baseHeight = -0.9
    basePosition = [0.25, -0.5, baseHeight]
    linkJointAxis = [[1,0,1]]
    
    centralLinkJointAxis = [[0,0,1]]
    centralBasePosition = [0.25, -1, baseHeight]
    
    upperLinkJointAxis = [[-1,0,1]]
    upperBasePosition = [0.25, -1.5, baseHeight]

    # Spawn deformable object
    deformable_obj_path = os.path.dirname(inspect.getfile(data_set)) + "/"
    soleBasePos = [0.4, 0, -0.2]
    # soleBaseOr = [1, 0, 1, 0]
    soleBaseOr = [0, 0, 0, 1]
    
    self._soleId = p.loadSoftBody(deformable_obj_path + "test.vtk", mass = 0.300, useNeoHookean = 1, NeoHookeanMu = 4000, NeoHookeanLambda = 1000, NeoHookeanDamping = .25, collisionMargin = 0.0006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 1, scale=1, basePosition=soleBasePos, baseOrientation=soleBaseOr)

    ############################ BOTTOM AREA ######################
    self._bottomLinkUid = p.createMultiBody(baseMass=baseMass,
                                            baseCollisionShapeIndex=boxId,
                                            baseVisualShapeIndex=baseVisualShapeId,
                                            basePosition=basePosition,
                                            baseOrientation=baseOrientation,
                                            linkMasses=linkMasses,
                                            linkCollisionShapeIndices=linkCollisionShapeIndices,
                                            linkVisualShapeIndices=linkVisualShapeIndices,
                                            linkPositions=linkPositions,
                                            linkOrientations=linkOrientations,
                                            linkInertialFramePositions=linkInertialFramePositions,
                                            linkInertialFrameOrientations=linkInertialFrameOrientations,
                                            linkParentIndices=linkParentIndices,
                                            linkJointTypes=linkJointTypes,
                                            linkJointAxis=linkJointAxis)
    
    # Change the link dynamics
    p.changeDynamics(self._bottomLinkUid,
                     -1,
                     linearDamping=damping,
                     lateralFriction=1,
                     jointDamping=damping)
    
    # Create bound between constrained cube and soft body nodes
    bottomAnchorPointsIndexes = [215, 258, 277]
    self._bottomAnchorsConstraintsIds = self._createSoftBodyAnchors(self._soleId, bottomAnchorPointsIndexes, self._bottomLinkUid, 0)

    # Get the initial axis length
    self._initialBottomAxisLength = self._currentAxisLength(self._bottomLinkUid)
    
    ############################ CENTRAL AREA ######################
    self._centralLinkUid = p.createMultiBody(baseMass=baseMass,
                                             baseCollisionShapeIndex=boxId,
                                             baseVisualShapeIndex=baseVisualShapeId,
                                             basePosition=centralBasePosition,
                                             baseOrientation=baseOrientation,
                                             linkMasses=linkMasses,
                                             linkCollisionShapeIndices=linkCollisionShapeIndices,
                                             linkVisualShapeIndices=linkVisualShapeIndices,
                                             linkPositions=linkPositions,
                                             linkOrientations=linkOrientations,
                                             linkInertialFramePositions=linkInertialFramePositions,
                                             linkInertialFrameOrientations=linkInertialFrameOrientations,
                                             linkParentIndices=linkParentIndices,
                                             linkJointTypes=linkJointTypes,
                                             linkJointAxis=centralLinkJointAxis)
    
    # Change the link dynamics
    p.changeDynamics(self._centralLinkUid,
                     -1,
                     linearDamping=damping,
                     lateralFriction=1,
                     jointDamping=damping)
    
    # Create bound between constrained cube and soft body nodes
    centralAnchorPointsIndexes = [236, 239, 264]
    self._centralAnchorsConstraintsIds = self._createSoftBodyAnchors(self._soleId, centralAnchorPointsIndexes, self._centralLinkUid, 0)

    # Get the initial axis length
    self._initialCentralAxisLength = self._currentAxisLength(self._centralLinkUid)
    
    ############################ UPPER AREA ######################
    self._upperLinkUid = p.createMultiBody(baseMass=baseMass,
                                           baseCollisionShapeIndex=boxId,
                                           baseVisualShapeIndex=baseVisualShapeId,
                                           basePosition=upperBasePosition,
                                           baseOrientation=baseOrientation,
                                           linkMasses=linkMasses,
                                           linkCollisionShapeIndices=linkCollisionShapeIndices,
                                           linkVisualShapeIndices=linkVisualShapeIndices,
                                           linkPositions=linkPositions,
                                           linkOrientations=linkOrientations,
                                           linkInertialFramePositions=linkInertialFramePositions,
                                           linkInertialFrameOrientations=linkInertialFrameOrientations,
                                           linkParentIndices=linkParentIndices,
                                           linkJointTypes=linkJointTypes,
                                           linkJointAxis=upperLinkJointAxis)
    
    # Change the link dynamics
    p.changeDynamics(self._upperLinkUid,
                     -1,
                     linearDamping=damping,
                     lateralFriction=1,
                     jointDamping=damping)
    
    # Create bound between constrained cube and soft body nodes
    upperAnchorPointsIndexes = [206,207,208,259]
    self._upperAnchorsConstraintsIds = self._createSoftBodyAnchors(self._soleId, upperAnchorPointsIndexes, self._upperLinkUid, 0)

    # Get the initial axis length
    self._initialUpperAxisLength = self._currentAxisLength(self._upperLinkUid)
    
    ##############################################################
    
    # Create bound between end-effector and soft body nodes
    contactPointAnchorPointsIndexes = [180, 252, 253, 274, 276]
    self._gripperAnchorsConstraintsIds = self._createSoftBodyAnchors(self._soleId, contactPointAnchorPointsIndexes, self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)

    
    # Close the gripper around the object
    # self.boxId = linkUid
    self.boxId = 0
    
    for i in range(10):
      self._kuka.closeGripper([0.,0.,0.,0.,0.05])
      p.stepSimulation()

    self._initialSolePosition = p.getBasePositionAndOrientation(self._soleId)[0]
    self._initialEndEffSoleDist = self._distance_end_eff_object(self._soleId)
    return self._observation()

  def _createSoftBodyAnchors(self, deformableObjectId, listOfAnchors, multiBodyId, indexOfBody):
    """
    Create anchor points between a list of mesh index points and a rigid body
    """
    constraintsIds = []
    for anchorIndex in listOfAnchors:
      constraintsIds.append(p.createSoftBodyAnchor(deformableObjectId, anchorIndex, multiBodyId, indexOfBody))
    return constraintsIds

  def _removeConstraints(self, constraintsIdsList):
    """
    Remove constraints from the simulator which ids are specified in constraintIdsList
    """
    for id in constraintsIdsList:
      p.removeConstraint(id)
  
  def _distance_end_eff_object(self, objectId):
    """
    Return distance between object specified by its ID and end effector of the robot
    """
    objectPosition = p.getBasePositionAndOrientation(objectId)[0]
    endEffPosition = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)[0]
    return np.linalg.norm(np.subtract(objectPosition, endEffPosition))

  def _distance_between_objects(self, objectId1, objectId2):
    """
    Return distance between two objects specified by their IDs
    """
    objectPosition1 = p.getBasePositionAndOrientation(objectId1)[0]
    objectPosition2 = p.getBasePositionAndOrientation(objectId2)[0]
    return np.linalg.norm(np.subtract(objectPosition1, objectPosition2))

  def _currentAxisLength(self, axisId):
    """
    Return current distance between the two cubes forming what we call an axis
    """
    movingCubePos =  p.getLinkState(axisId, 0)[0]
    baseCubePos = p.getBasePositionAndOrientation(axisId)[0]
    return np.linalg.norm(np.subtract(movingCubePos, baseCubePos))
  
  def _observation(self):
    """
    Return the current time.
    """
    return self._time

  def _reward(self):
    """
    Return the distance between the object and its initial pose
    """
    currentSolePosition = p.getBasePositionAndOrientation(self._soleId)[0]
    distance = np.linalg.norm(np.subtract(currentSolePosition, self._initialSolePosition))
    return distance

  def _infos(self):
    """
    Return some info about the task
    """
    infos = {}
    infos['bottom_axis_pos'] = p.getLinkState(self._bottomLinkUid, 0)[0]
    infos['central_axis_pos'] = p.getLinkState(self._centralLinkUid, 0)[0]
    infos['upper_axis_pos'] = p.getLinkState(self._upperLinkUid, 0)[0]
    infos['end_effector_pos'] = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)[0]
    infos['cube_pos'] = p.getBasePositionAndOrientation(self.boxId)[0]
    return infos

  def _update_sim(self):
    """
    Update the simulation parameters
    """
    time.sleep(self._timeStep)
    self._time += self._timeStep

    ### Handle the removal of axes if length has gone past the max axis length
    # Bottom Axis
    if(self._currentAxisLength(self._bottomLinkUid) - self._initialBottomAxisLength > self._bottomAxisLength):
      self._bottomAxisLength = math.inf
      self._removeConstraints(self._bottomAnchorsConstraintsIds)
      #print("Deleted bottom axis !")

    # Central Axis
    if(self._currentAxisLength(self._centralLinkUid) - self._initialCentralAxisLength > self._centralAxisLength):
      self._centralAxisLength = math.inf
      self._removeConstraints(self._centralAnchorsConstraintsIds)
      #print("Deleted central axis !")

    # Upper Axis
    if(self._currentAxisLength(self._upperLinkUid) - self._initialUpperAxisLength > self._upperAxisLength):
      self._upperAxisLength = math.inf
      self._removeConstraints(self._upperAnchorsConstraintsIds)
      #print("Deleted upper axis !")
      
    p.stepSimulation()

  def render(self):
    """
    Just renders the sim without applying any actions to it
    """
    self._update_sim()

  def setAxes(self, axes):
    """
    Set the axi(e)s to the given one(s) in arg
    """
    self._axes = axes

  def step(self,actions):
    """
    Action is the motor commands as outputed by the controller
    """
    end = False
    # Append to the motor commands the gripper joint position to have the cube in hand
    # gripperPose = [-0.499912, 0.000000, -0.000043, 0.499960, 0.000000, -0.000200]
    # motorCommands = np.concatenate((actions, gripperPose))
    # WARNING: need to set self._kuka.useInverseKinematics to false !
    self._kuka.applyAction(actions)
    self._kuka.closeGripper([0.,0.,0.,0.,0.05])
    
    ### Third step: evaluate the output of the behavior ###
    ### Return the distance between the original position of the cube and its last position ###
    self._update_sim()

    # Handle end cases
    endEffSoleDist = self._distance_end_eff_object(self._soleId)

    if((self._time > self._max_time)
       or (abs(endEffSoleDist - self._initialEndEffSoleDist) > 0.2)
       or (math.isinf(self._bottomAxisLength) and math.isinf(self._centralAxisLength) and math.isinf(self._upperAxisLength))):
      end = True
    
    return self._observation(), self._reward(), end, self._infos()

  
