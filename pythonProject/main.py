import server_req
import math_op
import time
import fuzzy_logic_python


def main_control_loop(turn_points, control, target, n):
    rotation = server_req.get_rotation(n)
    target_angle = math_op.get_target_angle(server_req.get_position(n), target, rotation)

    fuz_control = control.fuz_log([], target_angle, n)
    angle = fuz_control.get('angle')
    speed = fuz_control.get('speed')
    server_req.print_to_console(f'Control loop:\nAngle: {angle}\nSpeed: {speed}\n')

    if 0.00001 > angle > -0.00001:
        server_req.movement(speed, n)
    else:
        server_req.turn(angle, 2, n)
        server_req.movement(speed, n)
    turn_points.append(server_req.get_position(n))


if __name__ == "__main__":
    server_req.prepare_sim()

    for j in range(1):
        time.sleep(3)
        server_req.start_sim()
        turn_points = []

        #LR_arr = [0.6, 0.7, 0.6, 0.7, 1, 1.1, 1, 1.2]
        #LRF_arr = [0.5, 0.6, 0.55, 0.6, 1, 1.1, 1, 1.2]

        LR_arr = [1.1378750550921701, 1.2590445005349755, 1.25500513225747, 1.3158277513464467, 1.5062737632906598, 1.4645018439598694, 1.5730323677720999, 1.629882798085442]
        LRF_arr = [1.3900597868435844, 1.49282832926887, 1.5137960962878325, 1.6479956682329573, 1.8458763871153316, 1.861200935732206, 2.0262948675179078, 2.0446681673587523]
        control = fuzzy_logic_python.Fuzzy(LR_arr, LRF_arr)

        target_list = [[[0, -11]]]
        
        i = [0, 0, 0]
    
        n = 0
        start_time = time.time()

        while(True):
            target = target_list[n][i[n]]
            main_control_loop(turn_points, control, target, n)

            dist = math_op.get_target_dist(server_req.get_position(n), target)
            if dist < 0.1:
                i[n] += 1
                if i[n] == len(target_list[0]):
                    server_req.movement(0, n)
                    turn_points.append(server_req.get_position(n))
                    break


        dist = math_op.get_move_dist(turn_points)
        end_time = time.time() - start_time
        server_req.print_to_console(f'Robot num: {n}\nGOAL ACHIEVED\nWork time: {end_time}\nTravel distance: {dist}')

        server_req.stop_sim()
