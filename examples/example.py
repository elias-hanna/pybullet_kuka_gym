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

boxId = p.loadURDF(path + "cube.urdf", [0,3,2],useMaximalCoordinates = True)

basePos=[0,0,-2]
baseOr = [1, 0, 1, 0]
# Get data path
path = os.path.dirname(inspect.getfile(data_set)) + "/"

## Load soft object ##
bunnyId = p.loadSoftBody(path + "test.vtk", mass = 0.100, useNeoHookean = 1, NeoHookeanMu = 2000, NeoHookeanLambda = 600, NeoHookeanDamping = 0.25, collisionMargin = 0.0006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 1, scale=1, basePosition=basePos, baseOrientation=baseOr)


radius = 0.03

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
baseOrientation = [0, 0, 0, 1]

############################ BOTTOM AREA ######################
bottomRadius = 0.03

bottomBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[radius, radius, radius])

bottomMass = 0.01
bottomVisualShapeId = -1
bottomLink_Masses = [10000]
bottomLinkVisualShapeIndices = [-1]
bottomLinkPositions = [[0, 0, -0.11]]
bottomLinkOrientations = [[0, 0, 0, 1]]
bottomLinkInertialFramePositions = [[0, 0, 0]]
bottomLinkInertialFrameOrientations = [[0, 0, 0, 1]]
bottomIndices = [0]
bottomJointTypes = [p.JOINT_PRISMATIC]
bottomBaseOrientation = [0, 0, 0, 1]
bottomLinkCollisionShapeIndices = [bottomBoxId]
bottomAxis = [[1,0,1]]
bottomBasePosition = [0.25, -0.5, -1.85]

bottomLinkUid = p.createMultiBody(baseMass=bottomMass,
                                  baseCollisionShapeIndex=bottomBoxId,
                                  baseVisualShapeIndex=bottomVisualShapeId,
                                  basePosition=bottomBasePosition,
                                  baseOrientation=bottomBaseOrientation,
                                  linkMasses=bottomLink_Masses,
                                  linkCollisionShapeIndices=bottomLinkCollisionShapeIndices,
                                  linkVisualShapeIndices=bottomLinkVisualShapeIndices,
                                  linkPositions=bottomLinkPositions,
                                  linkOrientations=bottomLinkOrientations,
                                  linkInertialFramePositions=bottomLinkInertialFramePositions,
                                  linkInertialFrameOrientations=bottomLinkInertialFrameOrientations,
                                  linkParentIndices=bottomIndices,
                                  linkJointTypes=bottomJointTypes,
                                  linkJointAxis=bottomAxis)
# # Change the link dynamics
# p.changeDynamics(bottomLinkUid,
#                  -1,
#                  linearDamping=.1,
#                  lateralFriction=100000)

# Create bound between constrained cube and soft body nodes
# p.createSoftBodyAnchor(bunnyId, 215, bottomLinkUid, -1)
# p.createSoftBodyAnchor(bunnyId, 258, bottomLinkUid, -1)
# p.createSoftBodyAnchor(bunnyId, 277, bottomLinkUid, -1)
p.createSoftBodyAnchor(bunnyId, 215, -1, -1)
p.createSoftBodyAnchor(bunnyId, 258, -1, -1)
p.createSoftBodyAnchor(bunnyId, 277, -1, -1)
############################ CENTRAL AREA ######################
axis = [[1,0,0]]
basePosition = [0.25, -1, -1.85]

centralLinkUid = p.createMultiBody(baseMass=mass,
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
p.changeDynamics(centralLinkUid,
                 -1,
                 linearDamping=.1,
                 lateralFriction=100000)

## Create bound between constrained cube and soft body nodes
p.createSoftBodyAnchor(bunnyId  ,236,centralLinkUid,-1)
p.createSoftBodyAnchor(bunnyId  ,239,centralLinkUid,-1)
p.createSoftBodyAnchor(bunnyId  ,264,centralLinkUid,-1)

############################ UPPER AREA ######################
# axis = [[1,0,-1]]
# basePosition = [0.25, -1.5, -1.85]

# upperLinkUid = p.createMultiBody(baseMass=mass,
#                             baseCollisionShapeIndex=boxId,
#                             baseVisualShapeIndex=visualShapeId,
#                             basePosition=basePosition,
#                             baseOrientation=baseOrientation,
#                             linkMasses=link_Masses,
#                             linkCollisionShapeIndices=linkCollisionShapeIndices,
#                             linkVisualShapeIndices=linkVisualShapeIndices,
#                             linkPositions=linkPositions,
#                             linkOrientations=linkOrientations,
#                             linkInertialFramePositions=linkInertialFramePositions,
#                             linkInertialFrameOrientations=linkInertialFrameOrientations,
#                             linkParentIndices=indices,
#                             linkJointTypes=jointTypes,
#                             linkJointAxis=axis)
# # Change the link dynamics
# p.changeDynamics(upperLinkUid,
#                  -1,
#                  linearDamping=.1,
#                  lateralFriction=100000)

## Create bound between constrained cube and soft body nodes
# p.createSoftBodyAnchor(bunnyId  ,206,upperLinkUid,-1)
# p.createSoftBodyAnchor(bunnyId  ,207,upperLinkUid,-1)
# p.createSoftBodyAnchor(bunnyId  ,208,upperLinkUid,-1)
# p.createSoftBodyAnchor(bunnyId  ,259,upperLinkUid,-1)
p.createSoftBodyAnchor(bunnyId  ,206,-1,-1)
p.createSoftBodyAnchor(bunnyId  ,207,-1,-1)
p.createSoftBodyAnchor(bunnyId  ,208,-1,-1)
p.createSoftBodyAnchor(bunnyId  ,259,-1,-1)

# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=0, cameraPitch=-52, cameraTargetPosition=basePos)

while p.isConnected():
  p.setGravity(0,0,-10)
