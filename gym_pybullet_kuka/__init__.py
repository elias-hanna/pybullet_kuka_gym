import logging
from gym.envs.registration import register
from os.path import dirname, join

logger = logging.getLogger(__name__)

register(
    id='Velcro1DOF-v0',
    entry_point='gym_pybullet_kuka.velcro_1_dof:KukaVelcroObject'
)

register(
    id='Velcro-Deformable-v0',
    entry_point='gym_pybullet_kuka.velcro_deformable:KukaVelcroObject'
)
