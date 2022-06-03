import imp
import time
import joblib
import os
import os.path as osp
import tensorflow as tf
import torch
from spinup import EpochLogger
from spinup.utils.logx import restore_tf_graph
import gym
import matplotlib.pyplot as plt
import numpy as np
if __name__ == '__main__':
    # import argparse
    # parser = argparse.ArgumentParser()
    # parser.add_argument('fpath', type=str)
    # args = parser.parse_args()

    path = "/Users/lhx/lhx_ws/openAI/spinningup/data/ddpgpd/ddpgpd_s0/pyt_save/model.pt"
    model = torch.load(path)
    env = gym.make("Pendulum-v0")
    state = env.reset()
    anglog = []
    ratelog = []
    actlog = []
    while True:
        env.render()
        action = model.act(torch.torch.as_tensor(state, dtype=torch.float32))
        actlog.append(action)
        anglog.append(state[0])
        ratelog.append(state[2])
        state, reward, done, _ = env.step(action)
        if done:
            print("reward:", reward, "state:", state)
            env.close()
            break
    time = np.arange(len(actlog))
    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(time, ratelog, label='angular rate')
    plt.legend()
    plt.subplot(3,1,2)
    plt.plot(time, anglog, label='angular')
    plt.legend()
    plt.subplot(3,1,3)
    plt.plot(time, actlog, label='action')
    plt.legend()
    plt.show()
    