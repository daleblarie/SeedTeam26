import numpy as np
import json
import copy
import constants as c
from Formation2D import FORMATION
import datetime
import scipy.spatial.distance as distance
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
plt.rcParams['animation.ffmpeg_path'] = 'C:/FFmpeg/bin/ffmpeg.exe'

class SIMULATION():

    def __init__(self, form, t_steps=10):
        self.form=form
        self.t_steps = t_steps

        self.target_pos = np.zeros((2, t_steps))
        self.node_pos = np.zeros((2, form.N, t_steps))
        self.agent_pos = np.zeros((2, form.N, t_steps))

        self.active_nodes = np.zeros((1, t_steps))

        self.target_dist = np.zeros((1, t_steps))
        self.node_dist = np.zeros((form.N, t_steps * form.N))
        self.agent_dist = np.zeros((form.N, t_steps * form.N))

    def run(self, pb=False, save=False):
        self.run_simulation()
        # print('Sim target', self.target_dist[:2, -1])
        # print('Sim leader', self.agent_pos[:2, 2, 0], self.agent_pos[:2, 2, -1])
        if pb is False:
            self.run_animation(save)

    def run_simulation(self):

        for t in range(self.t_steps):
            dist = np.linalg.norm(self.form.agents[:2, self.form.leader] - self.form.target[:2, 0], axis=0)

            if dist <= self.form.r_t:
                self.form.circular_formation()
            else:
                self.form.V_Formation(2*math.pi/3)

            self.form.position_input()
            self.form.target_tracking_input()
            # self.form.collision_input()


            #form.nodal_input()
            # self.form.target_input()

            self.form.update_agent_position()
            # form.update_nodal_position()
            self.form.update_target_position()

            self.form.check_active_nodes()

            self.target_pos[:, t] = self.form.target[:2, 0]
            self.node_pos[:, :, t] = self.form.nodes[:2, :]
            self.agent_pos[:, :, t] = self.form.agents[:2, :]

            self.active_nodes[:, t] = sum(self.form.nodes[6, :])/self.form.N

            self.target_dist[:, t] = np.linalg.norm(self.agent_pos[:2, self.form.leader, t] - self.target_pos[:2, t], axis=0)
            self.node_dist[:, t * self.form.N:(t+1)*self.form.N] = distance.cdist(np.transpose(self.form.agents[:2, :]),  np.transpose(self.form.nodes[:2, :]), 'euclidean')
            self.agent_dist[:, t * self.form.N:(t+1)*self.form.N] = distance.cdist(np.transpose(self.form.agents[:2, :]),  np.transpose(self.form.agents[:2, :]), 'euclidean')

            # print(self.target_dist[:, t])

    def run_animation(self, save):
        writer = animation.FFMpegWriter()

        fig = plt.figure()
        # ax = plt.axes(xlim=(-100, 150), ylim=(-100, 100))
        ax = plt.axes(xlim=(-100, 500), ylim=(-300, 300))

        # ax = plt.axes()

        line3, = ax.plot([], [], 'g*')
        line1, = ax.plot([], [], 'ko')

        line2, = ax.plot([], [], 'r+')

        def init():
            line1.set_data([], [])
            line2.set_data([], [])
            line3.set_data([], [])
            return line1, line2, line3,

        def animate(i):
                line3.set_data(self.target_pos[0, i], self.target_pos[1, i])
                line2.set_data(self.agent_pos[0, :, i], self.agent_pos[1, :, i])
                line1.set_data(self.node_pos[0, :, i], self.node_pos[1, :, i])

                # xlim = (self.agent_pos[0, 2, i] - 100, self.agent_pos[0, 2, i] + 150)
                # ylim = (-100, 100)
                # ax = plt.axes(xlim=xlim, ylim=ylim)

                return line1, line2, line3,

        anim = FuncAnimation(fig, animate, init_func=init, frames=self.t_steps, interval=100, repeat=False, blit=False)

        if save is True:
            # fn = 'formation_' + str(datetime.datetime.now()) + '.avi'
            fn = 'formation.avi'
            anim.save(fn, writer=writer)

        plt.show()

if __name__ == '__main__':
    fileName = 'Formation_bestC0nstants.txt'
    with open(fileName, 'r') as json_file:
        last_line = json_file.readlines()[-1]
        k = json.loads(last_line)

    k = {int(k): v for k, v in k['k'].items()}

    N = c.N
    # initialPos = np.random.randint(-10, 10, (2, N))
    initialPos = np.asarray([[0, -20, 20, 0, 0], [0, 0, 0, 20, -20]])

    target = np.zeros((6, 1))
    # initial target position
    target[0, 0] = 100
    target[1, 0] = 0
    # initial target input
    target[4, 0] = 2



    # theta = self.form.theta_min()
    # self.form.V_Formation(theta, target)
    # self.form.circular_formation()

    form = FORMATION(N, initialPos, target, k=k)
    sim = SIMULATION(form, c.timeSteps)

    sim.run(pb=False, save=False)