# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie, sets the initial position/yaw
and flies a trajectory.
The initial pose (x, y, z, yaw) is configured in a number of variables and
the trajectory is flown relative to this position, using the initial yaw.
This example is intended to work with any absolute positioning system.
It aims at documenting how to take off with the Crazyflie in an orientation
that is different from the standard positive X orientation and how to set the
initial position of the kalman estimator.
"""
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from infoLog import LoggingExample

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

# Change the sequence according to your setup
#             x    y    z
sequence = [
    (0, 0, 0.7),
]


def wait_for_position_estimator(scf, logger):
    print('Waiting for estimator to find position...')


    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    for log_entry in logger:
        data = log_entry[1]

        var_x_history.append(data['kalman.varPX'])
        var_x_history.pop(0)
        var_y_history.append(data['kalman.varPY'])
        var_y_history.pop(0)
        var_z_history.append(data['kalman.varPZ'])
        var_z_history.pop(0)

        min_x = min(var_x_history)
        max_x = max(var_x_history)
        min_y = min(var_y_history)
        max_y = max(var_y_history)
        min_z = min(var_z_history)
        max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

        if (max_x - min_x) < threshold and (
                max_y - min_y) < threshold and (
                max_z - min_z) < threshold:
            break


def set_initial_position(cfIn, x, y, z, yaw_deg):
    cfIn.param.set_value('kalman.initialX', x)
    cfIn.param.set_value('kalman.initialY', y)
    cfIn.param.set_value('kalman.initialZ', z)

    # yaw_radians = math.radians(yaw_deg)
    # scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(cfIn, logger):
    cf = cfIn
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf, logger)


def run_sequence(cfIn, sequence, base_x, base_y, base_z, yaw):
    cf = cfIn

    for position in sequence:
        print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        for i in range(30):
            cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = LoggingExample(available[0][0], 'drone1')

        # Set these to the position and yaw based on how your Crazyflie is placed
        # on the floor
        initial_x = 1.0
        initial_y = 1.0
        initial_z = 0.0
        initial_yaw = 0  # In degrees
        # 0: positive X direction
        # 90: positive Y direction
        # 180: negative X direction
        # 270: negative Y direction

        # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set_initial_position(le._cf, initial_x, initial_y, initial_z, initial_yaw)
        # reset_estimator(le._cf, le)
        run_sequence(le._cf, sequence, initial_x, initial_y, initial_z, initial_yaw)
        time.sleep(50)
    else:
        print('No Crazyflies found, cannot run example')
