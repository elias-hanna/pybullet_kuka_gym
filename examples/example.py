import pybullet as p
from time import sleep
import pybullet_data
import os
import gym_pybullet_kuka.data as data_set

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

path = os.path.expanduser("~") + "/Documents/thesis/codes/preliminary_experiments/dev/bullet3/build_cmake/data/"
planeId = p.loadURDF(path + "plane.urdf", [0,0,-2])
# planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"plane.urdf"), [0,0,-2])

boxId = p.loadURDF(path + "cube.urdf", [0,3,2],useMaximalCoordinates = True)

# bunnyId = p.loadSoftBody(path + "torus.vtk", mass = 3, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)

basePos=[0,0,-2]
# Get data path
path = os.path.dirname(data_set.__file__) + "/"
# path = os.path.expanduser("~") + "/"
bunnyId = p.loadSoftBody(path + "test.vtk", mass = 0.100, useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.25, collisionMargin = 0.0006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 1, scale=1, basePosition=basePos)

# bunny2 = p.loadURDF(path + "torus_deform.urdf", [0,1,0], flags=p.URDF_USE_SELF_COLLISION)

# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setRealTimeSimulation(1)

p.resetDebugVisualizerCamera( cameraDistance=0.5, cameraYaw=0, cameraPitch=-52, cameraTargetPosition=basePos)

while p.isConnected():
  p.setGravity(0,0,-10)
