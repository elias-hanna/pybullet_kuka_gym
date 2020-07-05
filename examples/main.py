import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
import gym
import numpy as np
import pdb
import cv2
# from kuka_diverse_object_gym_env import KukaDiverseObjectEnv
from gym_pybullet_kuka.velcro_1_dof import KukaVelcroObject
# from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv

save_path = './data/train_image/'
def main():
    environment = KukaVelcroObject(renders=True, isDiscrete=False)
    for i in range(10):
        done = False
        index = 0
        environment.reset()
        # print(action)
        # pdb.set_trace()
        # while (not done):
        while (True):
            action = []
            # action = demo_policy_object._choose_action(low_state)
            action = [0., 0., 0., 0., 0.] 
            # print('action:',action)
            # state, reward, done, info = environment.step(action)
            environment.render()
            index += 1
            # print('step: ', index)
            # print('reward: ', reward)

            # image = environment._get_observation()

            # image_name = os.path.join(os.path.abspath(save_path), str(i) + str(index) + '.jpg')

            # cv2.imwrite(image_name, image)


if __name__=="__main__":
    main()
