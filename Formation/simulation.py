import numpy as np
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

        self.target_dist = np.zeros((1, t_steps))
        self.node_dist = np.zeros((form.N, t_steps * form.N))
        self.agent_dist = np.zeros((form.N, t_steps * form.N))

    def run(self, pb=False, save=False):
        self.run_simulation()

        if pb is False:
            self.run_animation(save)

    def run_simulation(self):

        for t in range(self.t_steps):
            dist = np.linalg.norm(self.form.agents[:2, self.form.leader] - self.form.target[:2, 0], axis=0)
            self.form.check_active_nodes()

            if dist <= self.form.r_t:
                self.form.circular_formation()
            else:
                self.form.V_Formation(2*math.pi/3)

            self.form.position_input()
            # self.form.target_tracking_input()
            # self.form.collision_input()

            #form.nodal_input()
            # self.form.target_input()

            self.form.update_agent_position()
            # form.update_nodal_position()
            self.form.update_target_position()

            self.target_pos[:, t] = self.form.target[:2, 0]
            self.node_pos[:, :, t] = self.form.nodes[:2, :]
            self.agent_pos[:, :, t] = self.form.agents[:2, :]

            self.target_dist[:, t] = np.linalg.norm(self.form.agents[:2, self.form.leader] - self.form.target[:2, 0], axis=0)
            self.node_dist[:, t * self.form.N:(t+1)*self.form.N] = distance.cdist(np.transpose(self.form.agents[:2, :]),  np.transpose(self.form.nodes[:2, :]), 'euclidean')
            self.agent_dist[:, t * self.form.N:(t+1)*self.form.N] = distance.cdist(np.transpose(self.form.agents[:2, :]),  np.transpose(self.form.agents[:2, :]), 'euclidean')

    def run_animation(self, save):
        writer = animation.FFMpegWriter()

        fig = plt.figure()
        ax = plt.axes(xlim=(-50, 1000), ylim=(-100, 100))
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

                return line1, line2, line3,

        anim = FuncAnimation(fig, animate, init_func=init, frames=self.t_steps, interval=100, repeat=False)

        if save is True:
            anim.save('formation.avi', writer=writer)

        plt.show()
