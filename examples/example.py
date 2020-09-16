import pybullet as p
from time import sleep
import pybullet_data
import os, inspect
import gym_pybullet_kuka.data as data_set

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

path = os.path.expanduser("~") + "/Documents/thesis/codes/preliminary_experiments/dev/bullet3/build_cmake/data/"
planeId = p.loadURDF(path + "plane.urdf", [0,0,-2])

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
damping = 0.01
linkMasses = [0.1]
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
basePosition = [0.25, -0.5, -1]
linkJointAxis = [[1,0,1]]

centralLinkJointAxis = [[0,0,1]]
centralBasePosition = [0.25, -1, -1]

upperLinkJointAxis = [[-1,0,1]]
upperBasePosition = [0.25, -1.5, -1]

# Deformable object parameters
basePos=[0,0,-2]
baseOr = [0, 0, 0, 1]

## SPAWN DEFORMABLE OBJECT ##

# Get data path
path = os.path.dirname(inspect.getfile(data_set)) + "/"

bunnyId = p.loadSoftBody(path + "test.vtk", mass = 0.5, useNeoHookean = 1, NeoHookeanMu = 4000, NeoHookeanLambda = 6000, NeoHookeanDamping = 0.25, collisionMargin = 0.0006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 1, scale=1, basePosition=basePos, baseOrientation=baseOr)

############################ BOTTOM AREA ######################
bottomLinkUid = p.createMultiBody(baseMass=baseMass,
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
p.changeDynamics(bottomLinkUid,
                 -1,
                 linearDamping=damping,
                 lateralFriction=1,
                 jointDamping=damping)

# Create bound between constrained cube and soft body nodes
p.createSoftBodyAnchor(bunnyId, 215, bottomLinkUid, 0)
p.createSoftBodyAnchor(bunnyId, 258, bottomLinkUid, 0)
p.createSoftBodyAnchor(bunnyId, 277, bottomLinkUid, 0)
# p.createSoftBodyAnchor(bunnyId, 215, -1, -1)
# p.createSoftBodyAnchor(bunnyId, 258, -1, -1)
# p.createSoftBodyAnchor(bunnyId, 277, -1, -1)
############################ CENTRAL AREA ######################
centralLinkUid = p.createMultiBody(baseMass=baseMass,
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
p.changeDynamics(centralLinkUid,
                 -1,
                 linearDamping=damping,
                 lateralFriction=1,
                 jointDamping=damping)

## Create bound between constrained cube and soft body nodes
p.createSoftBodyAnchor(bunnyId  ,236,centralLinkUid, 0)
p.createSoftBodyAnchor(bunnyId  ,239,centralLinkUid, 0)
p.createSoftBodyAnchor(bunnyId  ,264,centralLinkUid, 0)
# p.createSoftBodyAnchor(bunnyId  ,236,-1,-1)
# p.createSoftBodyAnchor(bunnyId  ,239,-1,-1)
# p.createSoftBodyAnchor(bunnyId  ,264,-1,-1)
############################ UPPER AREA ######################
upperLinkUid = p.createMultiBody(baseMass=baseMass,
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
p.changeDynamics(upperLinkUid,
                 -1,
                 linearDamping=damping,
                 lateralFriction=1,
                 jointDamping=damping)

# # Create bound between constrained cube and soft body nodes
p.createSoftBodyAnchor(bunnyId  ,206,upperLinkUid, 0)
p.createSoftBodyAnchor(bunnyId  ,207,upperLinkUid, 0)
p.createSoftBodyAnchor(bunnyId  ,208,upperLinkUid, 0)
p.createSoftBodyAnchor(bunnyId  ,259,upperLinkUid, 0)
# p.createSoftBodyAnchor(bunnyId  ,206,-1,-1)
# p.createSoftBodyAnchor(bunnyId  ,207,-1,-1)
# p.createSoftBodyAnchor(bunnyId  ,208,-1,-1)
# p.createSoftBodyAnchor(bunnyId  ,259,-1,-1)

# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=180, cameraPitch=-52, cameraTargetPosition=basePos)

while p.isConnected():
  p.setGravity(0,0,-10)
