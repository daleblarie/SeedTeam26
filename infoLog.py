# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import csv
import os

import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)



def convertDictToStr(dict):
    returnStr = ''
    firstPass = True
    for key in dict:
        if firstPass:
            returnStr += str(dict[key])
            firstPass = False
        else:
            returnStr += ','+ str(dict[key])

    return returnStr




class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, droneNum):
        """ Initialize and run the example with the specified link_uri """
        self.is_connected = False

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        self.link_uri = link_uri

        # Variable used to keep main loop occupied until disconnect
        self.droneNum = droneNum

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        try:
            os.mkdir("testCSV/" + str(self.droneNum))
        except Exception as e:
            print(e)

        try:
            print("openingFiles")
            f = open("./testCSV/" + str(self.droneNum) + "/stab.csv", 'w')
            f.write("stabilizer.roll, stabilizer.pitch, stabilizer.yaw\n")
            f.close()
            print("created stab")
        except Exception as e:
            print("stab csv not created")
            print(e)
            return

        try:
            f = open("./testCSV/" + str(self.droneNum) + "/gyro.csv", 'w')
            f.write("gyro.x, gyro.y, gyro.z\n")
            f.close()
        except:
            print("gyro csv not created")

        try:
            f = open("./testCSV/" + str(self.droneNum) + '/accel.csv', 'w')
            f.write("acc.x, acc.y, acc.z\n")
            f.close()
        except:
            print("acc csv not created")

        try:
            f = open("./testCSV/" + str(self.droneNum) + '/kalmanState.csv', 'w')
            f.write("kalman.stateX, kalman.stateY, kalman.stateZ\n")
            f.close()
        except:
            print("kalman csv not created")

        try:
            f = open("./testCSV/" + str(self.droneNum) + '/kalmanPos.csv', 'w')
            f.write("kalman.statePX, kalman.statePY, kalman.statePZ\n")
            f.close()
        except:
            print("kalman csv not created")

        try:
            f = open("./testCSV/" + str(self.droneNum) + '/kalmanD.csv', 'w')
            f.write("kalman.stateD0, kalman.stateD1, kalman.stateD2\n")
            f.close()
        except:
            print("kalman csv not created")

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='stab', period_in_ms=100)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')


        self._lg_gyro = LogConfig(name='gyro', period_in_ms=100)
        self._lg_gyro.add_variable('gyro.x', 'float')
        self._lg_gyro.add_variable('gyro.y', 'float')
        self._lg_gyro.add_variable('gyro.z', 'float')

        self._lg_accel = LogConfig(name='accel', period_in_ms=100)
        self._lg_accel.add_variable('acc.x', 'float')
        self._lg_accel.add_variable('acc.y', 'float')
        self._lg_accel.add_variable('acc.z', 'float')

        self._lg_kalmanState = LogConfig(name='kalmanState', period_in_ms=100)
        self._lg_kalmanState.add_variable('kalman.stateX', 'float')
        self._lg_kalmanState.add_variable('kalman.stateY', 'float')
        self._lg_kalmanState.add_variable('kalman.stateZ', 'float')

        self._lg_kalmanPos = LogConfig(name='kalmanPos', period_in_ms=100)
        self._lg_kalmanPos.add_variable('kalman.statePX', 'float')
        self._lg_kalmanPos.add_variable('kalman.statePY', 'float')
        self._lg_kalmanPos.add_variable('kalman.statePZ', 'float')

        self._lg_kalmanD = LogConfig(name='kalmanD', period_in_ms=100)
        self._lg_kalmanD.add_variable('kalman.stateD0', 'float')
        self._lg_kalmanD.add_variable('kalman.stateD1', 'float')
        self._lg_kalmanD.add_variable('kalman.stateD2', 'float')





        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_gyro)
            self._cf.log.add_config(self._lg_accel)
            self._cf.log.add_config(self._lg_kalmanState)
            self._cf.log.add_config(self._lg_kalmanPos)
            self._cf.log.add_config(self._lg_kalmanD)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_gyro.data_received_cb.add_callback(self._stab_log_data)
            self._lg_accel.data_received_cb.add_callback(self._stab_log_data)
            self._lg_kalmanState.data_received_cb.add_callback(self._stab_log_data)
            self._lg_kalmanPos.data_received_cb.add_callback(self._stab_log_data)
            self._lg_kalmanD.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            self._lg_gyro.error_cb.add_callback(self._stab_log_error)
            self._lg_accel.error_cb.add_callback(self._stab_log_error)
            self._lg_kalmanState.error_cb.add_callback(self._stab_log_error)
            self._lg_kalmanPos.error_cb.add_callback(self._stab_log_error)
            self._lg_kalmanD.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
            self._lg_gyro.start()
            self._lg_accel.start()
            self._lg_kalmanState.start()
            self._lg_kalmanPos.start()
            self._lg_kalmanD.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        self.connected = True
        t = Timer(5, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        with open('testCSV/' + str(self.droneNum)+ "/" + logconf.name + '.csv','a') as fd:
            fd.write(convertDictToStr(data) + "\n")
            fd.close()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = LoggingExample(available[0][0], 'drone1')
        # le1 = LoggingExample(available[0][0], 'gyro')
    else:
        print('No Crazyflies found, cannot run example')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(10)
