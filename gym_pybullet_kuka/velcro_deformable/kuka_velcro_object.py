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
    # p.resetSimulation()
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

    # Add the object that needs to be manipulated
    radius = 0.03
    # sphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[radius, radius, radius])

    mass = 0.01
    visualShapeId = -1
    link_Masses = [10000]
    linkCollisionShapeIndices = [boxId]
    linkVisualShapeIndices = [-1]
    linkPositions = [[0, 0, -0.11]]
    linkOrientations = [[0, 0, 0, 1]]
    linkInertialFramePositions = [[0, 0, 0]]
    linkInertialFrameOrientations = [[0, 0, 0, 1]]
    indices = [0]
    jointTypes = [p.JOINT_PRISMATIC]
    axis = self._axes
    basePosition = [0.6, 0, -.05]
    baseOrientation = [0, 0, 0, 1]

    linkUid = p.createMultiBody(baseMass=mass,
                                baseCollisionShapeIndex=boxId,
                                baseVisualShapeIndex=visualShapeId,
                                basePosition=basePosition,
                                baseOrientation=baseOrientation,
                                linkMasses=link_Masses,
                                linkCollisionShapeIndices=linkCollisionShapeIndices,
                                linkVisualShapeIndices=linkVisualShapeIndices,
                                linkPositions=linkPositions,
                                linkOrientations=linkOrientations,
                                linkInertialFramePositions=linkInertialFramePositions,
                                linkInertialFrameOrientations=linkInertialFrameOrientations,
                                linkParentIndices=indices,
                                linkJointTypes=jointTypes,
                                linkJointAxis=axis)
    # Change the link dynamics
    p.changeDynamics(linkUid,
                     -1,
                     linearDamping=.1,
                     lateralFriction=100000)

    # Spawn deformable object
    deformable_obj_path = os.path.dirname(inspect.getfile(data_set)) + "/"
    soleBasePos = [0.9, 0, 0]
    soleBaseOr = [1, 0, 1, 0]
    soleId = p.loadSoftBody(deformable_obj_path + "test.vtk", mass = 0.100, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.25, collisionMargin = 0.0006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 1, scale=1, basePosition=soleBasePos, baseOrientation=soleBaseOr)
    # p.createSoftBodyAnchor(linkUid, 
    # path = os.path.expanduser("~") + "/Documents/thesis/codes/preliminary_experiments/dev/bullet3/build_cmake/data/"

    # bunnyId = p.loadSoftBody(path + "bread.vtk", mass = 1.5, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.1, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.1, scale=0.2, basePosition= [0.9, 0, 0.05], baseOrientation= [1,0,0,1])

    # Close the gripper around the object
    self.boxId = linkUid
    
    for i in range(10):
      self._kuka.closeGripper([0.,0.,0.,0.,0.05])
      # time.sleep(self._timeStep*2)
      # self._kuka.
      p.stepSimulation()

    self.initialBoxPosition = p.getBasePositionAndOrientation(self.boxId)[0]
    self.initialEndEffBoxDist = self._distance_end_eff_object(self.boxId)
    # print("INITIAL BOX POSE", self.initialBoxPosition)
    return self._observation()

  def _distance_end_eff_object(self, objectId):
    objectPosition = p.getBasePositionAndOrientation(objectId)[0]
    endEffPosition = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)[0]
    return np.linalg.norm(np.subtract(objectPosition, endEffPosition))

  def _observation(self):
    """
    Return the current time.
    """
    return self._time

  def _reward(self):
    """
    Return the distance between the object and its initial pose
    """
    currentBoxPosition = p.getBasePositionAndOrientation(self.boxId)[0]
    distance = np.linalg.norm(np.subtract(currentBoxPosition, self.initialBoxPosition))
    return distance

  def _infos(self):
    """
    Return some info about the task
    """
    infos = {}
    infos['cube_pos'] = p.getBasePositionAndOrientation(self.boxId)[0]
    return infos

  def _update_sim(self):
    """
    Update the simulation parameters
    """
    # time.sleep(self._timeStep)
    self._time += self._timeStep
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
    endEffBoxDist = self._distance_end_eff_object(self.boxId)
    
    if((self._time > self._max_time) or (abs(endEffBoxDist - self.initialEndEffBoxDist) > 0.02)):
      end = True
    
    return self._observation(), self._reward(), end, self._infos()

  
