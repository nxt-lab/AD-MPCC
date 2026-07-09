import sys, os
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _root)
os.chdir(_root)

from dataclasses import dataclass

import time
import yaml
import gym
from argparse import Namespace
from regulators.pure_pursuit import *
from regulators.path_follow_diff_mpcc_casadi import *
from regulators.get_look_table import *
from PAIL_MPCC.dynamic import DynamicBicycleModel
from PAIL_MPCC.closest_point import *
import numpy as np

import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.optimize import minimize_scalar
import pandas as pd
from scenarios import *
from common import (
    MPCConfigDYNBase, DrawDebug, ground_truth_pacejka, make_render_callback,
    make_log_dict, apply_slip_friction, compute_stage_weights,
)
from diff_look import lookup_q


@dataclass
class MPCConfigDYN(MPCConfigDYNBase):
    q_theta: float = 300.0


def main():
    model_to_use = 'dynamic'
    map_name = 'Oschersleben'
    rotate_map = True
    slip_mode = True

    control_step = 100.0
    render_every = 200.0
    constant_speed = False
    constant_speed_value = 8.0
    velocity_profile_multiplier = 0.9


    dyn_config = MPCConfigDYN()
    work = {'mass': 1225.88, 'lf': 0.80597534362552312, 'tlad': 10.6461887897713965, 'vgain': 1.0}

    with open('configs/config_%s.yaml' % map_name) as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)


    raceline = np.loadtxt(conf.wpt_path, delimiter=";", skiprows=3)
    waypoints = np.array(raceline)

    waypoints[:, 1] *= dyn_config.scale
    waypoints[:, 2] *= dyn_config.scale
    if rotate_map == True:
        waypoints[:, 3] += 1.5707963268

    if constant_speed:
        waypoints[:, 5] = np.ones((waypoints[:, 5].shape[0],)) * constant_speed_value
    else:
        waypoints[:, 5] *= velocity_profile_multiplier

    planner_pp = PurePursuitPlanner(conf, 0.805975 + 1.50876)
    planner_pp.waypoints = waypoints

    ini = np.array([[waypoints[start_point, 1], waypoints[start_point, 2], (waypoints[start_point, 3]
                        + np.pi) % (2*np.pi) - np.pi, 0.0, v_x_init, 0.0, 0.0]])
    planner_dyn_mpc = STMPCCPlannerCasadi(model=DynamicBicycleModel(config=dyn_config), waypoints=waypoints,
                                   config=dyn_config, index=start_point)
    find_theta = ThetaLookupTable(spline_x, spline_y, theta_min, theta_max, n_samples=10000000)
    draw = DrawDebug()

    render_callback = make_render_callback(planner_pp, draw)

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext,
                   num_agents=1, timestep=0.001, model='MB', drive_control_mode='acc',
                   steering_control_mode='vel')

    env.add_render_callback(render_callback)
    obs, step_reward, done, info = env.reset(
        ini)
    env.render()

    laptime = 0.0
    start = time.time()
    last_render = 0

    log = make_log_dict()

    num_of_sim_steps = int(control_step / (env.timestep * 1000.0))


    u = [0.0, 0.0]
    counter = 0
    while not done:


        vehicle_state = np.array([env.sim.agents[0].state[0],
                                  env.sim.agents[0].state[1],
                                  env.sim.agents[0].state[3],
                                  env.sim.agents[0].state[4] ,
                                  env.sim.agents[0].state[10],
                                  env.sim.agents[0].state[5],
                                  env.sim.agents[0].state[2],
                                  ]) + np.random.randn(7)

        if len(log['theta']) == 0:
                    theta_cur = find_theta.query_near_prev(
                        env.sim.agents[0].state[0],
                        env.sim.agents[0].state[1],
                        0.0,
                        k_neighbors=20,
                        forward_only=True,
                    )
        else:
            theta_cur = find_theta.query_near_prev(
                env.sim.agents[0].state[0],
                env.sim.agents[0].state[1],
                log['theta'][-1],
                k_neighbors=20,
                forward_only=True,
            )


        K_gt = ground_truth_pacejka(dyn_config.MASS)
        BR, CR, DR, BF, CF, DF, CM = K_gt

        start_compute_time = time.time()

        weights = compute_stage_weights(dyn_config, vehicle_state, theta_cur, K_gt, theta_min, theta_max, round_theta, lookup_q)

        u, mpc_ref_path_x, mpc_ref_path_y, mpc_pred_x, mpc_pred_y, _ = planner_dyn_mpc.plan(vehicle_state, K_gt, weights)
        end_compute_time = time.time()
        u[0] = u[0] / planner_dyn_mpc.config.MASS
        print("vx:", env.sim.agents[0].state[3])
        print("vy:", env.sim.agents[0].state[10])
        print("acceleration:", u[0])

        draw.reference_traj_show = np.array([mpc_ref_path_x, mpc_ref_path_y]).T
        draw.predicted_traj_show = np.array([mpc_pred_x, mpc_pred_y]).T

        _, tracking_error, _, _, n_point = nearest_point_on_trajectory(np.array([env.sim.agents[0].state[0], env.sim.agents[0].state[1]]),
                                                                       np.array([waypoints[:, 1], waypoints[:, 2]]).T)

        if slip_mode == True:
            apply_slip_friction(env, theta_cur, constant_friction, weight_slip, weight_slip_2,
                                 time_slip_start, time_slip_end, time_slip_start_2, time_slip_end_2,
                                 time_slip_start_3, time_slip_end_3, track_length=track_length)
        step_reward = 0.0
        sim_time = 0.0

        log['time_compute'].append(end_compute_time - start_compute_time)
        log['time'].append(laptime)
        log['x'].append(env.sim.agents[0].state[0])
        log['y'].append(env.sim.agents[0].state[1])
        log['vx'].append(env.sim.agents[0].state[3])
        log['vy'].append(env.sim.agents[0].state[10])
        log['yaw'].append(env.sim.agents[0].state[4])
        log['yaw_rate'].append(env.sim.agents[0].state[5])
        log['steer_angle'].append(env.sim.agents[0].state[2])
        log['theta'].append(theta_cur)
        log['acce'].append(u[0])
        log['steering_rate'].append(u[1])


        for i in range(num_of_sim_steps):
            obs, rew, done, info = env.step(np.array([[u[1], u[0]]]))
            step_reward += rew
            sim_time += env.timestep

            last_render += 1
            if last_render >= render_every:
                last_render = 0
                env.render(mode='human_fast')

        laptime += step_reward

        log['x_ref'].append(waypoints[:, 1][n_point])
        log['y_ref'].append(waypoints[:, 2][n_point])
        log['v_ref'].append(waypoints[:, 5][n_point])
        log['tracking_error'].append(tracking_error)
        log['lap_n'].append(obs['lap_counts'][0])
        log['BR'].append(BR)
        log['CR'].append(CR)
        log['DR'].append(DR)
        log['BF'].append(BF)
        log['CF'].append(CF)
        log['DF'].append(DF)
        log['CM'].append(CM)

        log['mu_x'].append(env.params['tire_p_dx1'])
        log['mu_y'].append(env.params['tire_p_dy1'])

        if tracking_error > 4.0:
            done = 1
            break
        if obs['lap_counts'][0] == number_of_laps:
            done = 1
    print('Lap finished! Lap time: %.2f seconds' % laptime)
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)
    print('Average compute time per step: %.4f seconds' % np.mean(log['time_compute']))
    os.makedirs('results', exist_ok=True)
    with open(os.path.join('results', f'diff_mpcc_scale{dyn_config.scale}_Tk{dyn_config.TK}_log_{map_name}_full_Vinit_{v_x_init}_c{dyn_config.q_contour}_l{dyn_config.q_lag}_p{dyn_config.q_theta}_friction{constant_friction}_weight{weight_slip}_slip_{time_slip_start}_{time_slip_end}_{time_slip_start_2}_{time_slip_end_2}_{time_slip_start_3}_{time_slip_end_3}'), 'w') as f:
        json.dump(log, f)


if __name__ == '__main__':
    main()
