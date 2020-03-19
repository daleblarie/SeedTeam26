import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils.multiranger import Multiranger


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
     (0, 0, 0.5, 0),
     (1, 0, 0.5, 0),
     (0, 0, 0.5, 0),
     (0, 0, 0, 0)
    
    # (0, 0, 0.5, 0),
    # (0, 0, 0, 0)


]

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
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
            
            
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)
 
    
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))
    
    
def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

## Multiranger Deck Part
def is_close(range):
    MIN_DISTANCE = 0.4  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE
    
def fly_away(scf):

    VELOCITY = 0.5
    velocity_x = 0.0
    velocity_y = 0.0

    if is_close(multiranger.front):
    # if is_close(multiranger.up):

        velocity_x -= VELOCITY
    if is_close(multiranger.back):
        velocity_x += VELOCITY
    if is_close(multiranger.left):
        velocity_y -= VELOCITY
    if is_close(multiranger.right):
        velocity_y += VELOCITY

    #     keep_flying = False

    #motion_commander.start_linear_motion(velocity_x, velocity_y, 0)
    time.sleep(0.1)


    
    
def run_sequence(scf, sequence):
    cf = scf.cf
    
    VELOCITY = 0.5
    velocity_x = 0.0
    velocity_y = 0.0

    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(50):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)
            if is_close(multiranger.front):
                velocity_x -= VELOCITY
                motion_commander.back(0.3)
                time.sleep(0.1)
            if is_close(multiranger.back):
                velocity_x += VELOCITY
                motion_commander.forward(0.3)
                time.sleep(0.1)
            if is_close(multiranger.left):
                velocity_y -= VELOCITY
                motion_commander.right(0.3)
                time.sleep(0.1)
            if is_close(multiranger.right):
                velocity_y += VELOCITY
                motion_commander.left(0.3)
                time.sleep(0.1)
                
                
                
            #time.sleep(0.1) #COMMENT
           # motion_commander.start_linear_motion(velocity_x, velocity_y, 0)
            #time.sleep(0.1) #UNCOMMENT

    cf.commander.send_stop_setpoint()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    
    
    
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                reset_estimator(scf)
                start_position_printing(scf)
                run_sequence(scf, sequence)


