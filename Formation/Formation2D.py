import numpy as np
import math
import copy


class FORMATION:
    def __init__(self, numAgents, InitialPos, targetInit, k):
        self.N = numAgents
        self.target = copy.copy(targetInit)
        self.nodes = np.zeros((7, self.N))
        self.agents = np.zeros((6, self.N))
        self.agents[:2, :] = InitialPos

        self.leader = None
        self.leader_selection()
        # self.nodes[:2, self.leader] = np.zeros(2)

        # self.k = {'k': [.1, .05], 'o': [90, 15], 't': [1, 0.6, 4], 'j': [.1, .4, 1], 'v': [.7, 1.2, 0.2]}
        # self.epsilon = {'a': 1, 'b': 0.5, 'c': 0.7}
        # self.lamb = {'a': 15, 'b': 9, 'c': 5}

        self.k = k
        # self.epsilon = {'a': 1, 'b': 0.5, 'c': 0.7}
        # self.lamb = {'a': 15, 'b': 9, 'c': 5}                                                               # repulsive radius

        self.r_r = 20
        self.r_t = 70
        self.r_tau = 50
        # attractive radius
        self.r_a = 10
        self.dist = 80

    def leader_selection(self):
        dist = np.linalg.norm(self.agents[:2, :] - self.target[:2, :], axis=0)
        ind = np.argmin(dist)
        self.nodes[6, 0] = 1
        self.leader = ind

    def rotation(self, theta, direction='cw'):

        if direction == 'cw':
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
        else:
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, s), (-s, c)))
        return R.reshape(2, 2)

    def update_target_position(self):
        self.target[:2, :] += self.target[2:4, :]
        self.target[2:4, :] += self.target[4:6, :]
        self.target[4:6, :] = np.zeros((2, 1))

    def target_input(self, input=np.asarray([[0], [0]])):
        self.target[4:6, :] = input

    def update_agent_position(self):
        self.agents[:2, :] += self.agents[2:4, :]
        self.agents[2:4, :] += self.agents[4:6, :]

    def update_nodal_position(self):
        self.nodes[:2, :] += self.nodes[2:4, :]
        self.nodes[2:4, :] += self.nodes[4:6, :]

    def nodal_input(self):
         # dist = np.linalg.norm(self.target[:2, :] - self.nodes[:2, [0]])
         # n = (self.nodes[:2, [0]] - self.target[:2, :]) / dist
         #
         # if dist < self.r_t:
         #     ft = (((1/dist) - (1/self.r_tau)) * (self.k['t'][1]/dist**2) - (self.k['t'][2]*(dist - self.r_tau))/(self.r_t - self.r_tau)) * n
         # else:
         #     ft = -self.k['t'][0] * n
         #
         # self.nodes[4:6, :] += ft - self.k['v'][1] * (self.nodes[2:4, [0]] - self.target[2:4, :]) + self.target[4:6, :]
         self.nodes[4:6, :] = self.agents[4:6, [self.leader]] * np.ones((2, self.N))

    def theta_min(self):
        return math.acos(1-(self.r_r**2)/(2*self.dist**2))

    def rotation_leater_target(self):
        vec1 = (self.target[:2, :] - self.agents[:2, [self.leader]]) / (
            np.linalg.norm(self.target[:2, :] - self.agents[:2, [self.leader]]))
        vec2 = np.array([[1], [0]])

        vec1 = vec1.reshape(2, )
        vec2 = vec2.reshape(2, )

        dot_product = np.dot(vec1, vec2)
        # dot_product = vec1 * vec2
        theta2 = np.arccos(dot_product)

        R = self.rotation(theta2, 'cw')

        return R

    def V_Formation(self, theta, bn=np.array([[5], [5]])):
        # Generates a v shaped formation based on the position
        # of the leader and the target
        bn = self.dist # * np.array([[math.cos(theta / 2)], [math.sin(theta / 2)]])

        qmup = bn * np.array([[math.cos(theta / 2)], [math.sin(theta / 2)]])
        qnp =bn * np.array([[math.cos(math.pi - (theta / 2))], [math.sin(math.pi - (theta / 2))]])

        q_mup = qmup.reshape((2,))
        q_np = qnp.reshape((2,))

        R = self.rotation_leater_target()

        q_mu = (R @ q_mup)
        q_n = (R @ q_np)

        for j in range(0, self.N):
            if j <= self.N/2:
                e1 = j 
                self.nodes[:2, j] = self.agents[:2, self.leader] - e1 * q_mu
            else:
                e2 = j - math.floor(self.N/2)
                self.nodes[:2, j] = self.agents[:2, self.leader] + e2 * q_n

    def circular_formation(self):
        dist = np.linalg.norm(self.agents[:2, self.leader] - self.target[:2, 0], axis=0)
        R = self.rotation_leater_target()
        for j in range(self.N):
            self.nodes[:2, [j]] = self.target[:2, :] - R * dist @ np.array([[math.cos(2*j*math.pi/self.N)], [math.sin(2*j*math.pi/self.N)]])

    def check_active_nodes(self):
        active = np.zeros((1, self.N))

        for i in range(self.N):
            dist = np.linalg.norm(self.agents[:2, [i]] - self.nodes[:2, :], axis=0)
            ind = np.argmin(dist)
            dist = dist[ind]

            if dist <= self.r_a:
                active[:, ind] = 1

        self.nodes[6, :] = active

    def position_input(self):
        
        input = np.zeros((2, self.N))

        self.nodes[6, 1:] = np.zeros((1, self.N-1))

        for i in np.arange(self.N):
            if i != self.leader:
                dist = np.linalg.norm(self.agents[:2, [i]] - self.nodes[:2, :], axis=0)
                ind = np.argmin(dist)
                dist = dist[ind]

                # ind += 1

                if dist <= self.r_a and self.nodes[6, ind-1] == 1:
                    input[:, i] = -self.k[3][0]*(self.agents[:2, i] - self.nodes[:2, ind]) - self.k[4][0]*(self.agents[2:4, i]-self.nodes[2:4, ind]) + self.nodes[4:6, ind]
                    self.nodes[6, ind] = 1

                elif dist <= self.r_a and self.nodes[6, ind-1] == 0:
                    input[:2, i] = -self.k[3][1] * (self.agents[:2, i] - self.nodes[:2, ind-1]) - self.k[4][0] * (self.agents[2:4, i] - self.nodes[2:4, ind-1]) + self.nodes[4:6, ind-1]
                    self.nodes[6, ind-1] = 1

                else:
                    if self.nodes[6, ind] == 0:
                        norm = (self.agents[:2, i] - self.nodes[:2, ind]) / dist
                        input[:, i] = -self.k[3][2]*norm - self.k[4][0]*(self.agents[2:4, i] - self.nodes[2:4, ind]) + self.nodes[4:6, ind]
                        self.nodes[6, ind] = 1

                    else:
                        free_nodes = np.argwhere(self.nodes[6, :] == 0)[0]
                        dist = np.linalg.norm(self.agents[:2, [i]] - self.nodes[:2, free_nodes], axis=0)
                        min_dist = np.argmin(dist)
                        ind = free_nodes[min_dist]
                        dist = dist[min_dist]

                        norm = (self.agents[:2, i] - self.nodes[:2, ind]) / dist
                        input[:, i] = -self.k[3][2] * norm - self.k[4][0] * (
                                    self.agents[2:4, i] - self.nodes[2:4, ind]) + self.nodes[4:6, ind]
                        self.nodes[6, ind] = 1
        # print(input)
        self.agents[4:6, :] = input

    def collision_input(self):
        input = np.zeros((2, self.N))
        for i in range(self.N):
            ui = 0
            col = np.hstack((self.agents, self.target))
            for k in range(self.N+1):
                if i != k & i != self.leader & k != self.leader:
                    dist = np.linalg.norm(self.agents[:2, i] - col[:2, k], axis=0)
                    norm = (self.agents[:2, i] - col[:2, k]) / dist

                    c = 0
                    if dist <= self.r_r:
                        c = 1

                    fk = (((1/dist) - (1/self.r_r)) * (self.k[0][0]/(dist**2)) - self.k[0][1] * (dist - self.r_r)) * c * norm
                    ui += (fk - c*self.k[4][2]*(self.agents[2:4, i] - col[2:4, k]))

                    input[:, i] = ui
        self.agents[4:6, :] += input

    def target_tracking_input(self):
        dist = np.linalg.norm(self.target[:2, :] - self.agents[:2, [self.leader]])
        n = (self.agents[:2, [self.leader]] - self.target[:2, :]) / dist

        if dist < self.r_t:
            ft = (((1/dist) - (1/self.r_tau)) * (self.k[2][1]/(dist**2)) - (self.k[2][2]*(dist - self.r_tau))/(self.r_t - self.r_tau)) * n
        else:
            ft = -self.k[2][0] * n

        self.agents[4:6, [self.leader]] += ft - self.k[2][1] * (self.agents[2:4, [self.leader]] - self.target[2:4, :]) + self.target[4:6, :]


if __name__ == '__main__':
    # target = np.random.randint(-40, 40, (2, 1))
    target = np.zeros((6, 1))
    target[0, 0] = 100
    target[1, 0] = 0
    target[4, 0] = 1
    # theta = 2*math.pi/3
    # theta = math.radians(36.87 * 2)
    N = 5
    # initialPos = np.random.randint(-5, 5, (2, N))
    initialPos = np.asarray([[0, 0, 0, 0, 0], [0, -2, -4, -6, -8]])
    k = {0: [.1, .05], 1: [90, 15], 2: [1, 0.6, 4], 3: [.1, .4, 1], 4: [.7, 1.2, 0.2]}
    form = FORMATION(N, initialPos, target, k)
    theta = form.theta_min()
    form.V_Formation(theta, target)
    # form.circular_formation()
    print(form.nodes)

    # form.nodal_input()
    # run_simulation(form, 100, pb=False)
