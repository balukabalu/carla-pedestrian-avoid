import glob
import os
import sys

import time
import numpy as np
import random
import argparse
from keras.models import model_from_json, Model
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import Adam
import tensorflow as tf
from keras.engine.training import collect_trainable_weights
import json

from ReplayBuffer import ReplayBuffer
from ActorNetwork import ActorNetwork
from CriticNetwork import CriticNetwork
from OU import OU
import timeit



try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass



import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from carla_env import CarlaEnv

try:
    print(sys.path)
    sys.path.append(glob.glob('../'))
    print(sys.path)
except IndexError:
    pass


OU = OU()       #Ornstein-Uhlenbeck Process
    
    

def main(train_indicator = 0):


# ==============================================================================
# -- Neural Network Setup ----------------------------------------------------
# ==============================================================================
    BUFFER_SIZE = 100000
    BATCH_SIZE = 32
    GAMMA = 0.99
    TAU = 0.001     #Target Network HyperParameters
    LRA = 0.0001    #Learning rate for Actor
    LRC = 0.001     #Lerning rate for Critic

    action_dim = 1  #Steering/Acceleration/Brake
    state_dim = 9  #of sensors input


    EXPLORE = 100000.
    episode_count = 1500
    max_steps = 5000
    reward = 0
    done = False
    step = 0
    epsilon = 1
    indicator = 0
    prev_step = 0
    colsens = None

    #Tensorflow GPU optimization
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    sess = tf.Session(config=config)
    from keras import backend as K
    K.set_session(sess)

    actor = ActorNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRA)
    critic1 = CriticNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRC)
    critic2 = CriticNetwork(sess, state_dim, action_dim, BATCH_SIZE, TAU, LRC)
    buff = ReplayBuffer(BUFFER_SIZE)    #Create replay buffer
    print("Now we load the weight")
    actor_1 =  "actormodel.h5"
    critic_11 = "criticmodel1.h5"
    critic_12 = "criticmodel2.h5"
    actor_2 =  "actormodel.json"
    critic_21 =  "criticmodel1.json"
    critic_22 =  "criticmodel2.json"
    actual = "_801"
    try:
        actor.model.load_weights(actual+"actormodel.h5")
        critic1.model.load_weights(actual+"criticmodel1.h5")
        critic2.model.load_weights(actual+"criticmodel2.h5")
        actor.target_model.load_weights(actual+"actormodel.h5")
        critic1.target_model.load_weights(actual+"criticmodel1.h5")
        critic2.target_model.load_weights(actual+"criticmodel2.h5")
        print("Weight load successfully")
    except:
        print("Cannot find the weight")

    env = CarlaEnv()

    env.start2()

# ==============================================================================
# -- Episodes and steps ----------------------------------------------------
# ==============================================================================

    for i in range(episode_count):

        print("Episode : " + str(i) + " Replay Buffer " + str(buff.count()))
        if i != 0:
            env.kill()

        env.actor_list = []
        env.setup_car() 
        env.setup_pedestrian()
        world_snapshot = env.world.get_snapshot()
        cur_map = env.world.get_map()
        lat_err = 0
        ang_err = 0
        speedX = 0
        speedY = 0
        ang_vel = 0
        accX = 0 
        accY = 0
        d_ped_veh = 0
        angle_ped_veh =0
        a_t = np.zeros([1, action_dim])
        spl = 0   #start_pedestrian_learning

        s_t = np.hstack((lat_err, ang_err, speedX, speedY, ang_vel, accX, accY, d_ped_veh, angle_ped_veh))	#initial states
        a_t0 = np.zeros([1, action_dim])
        noise_t = np.zeros([1,action_dim])
     
        total_reward = 0.
        for j in range(max_steps):
            loss = 0 
            loss1 = 0
            loss2 = 0
            epsilon -= 1.0 / EXPLORE

            a_t = np.zeros([1,action_dim])          
            a_t_original = actor.model.predict(s_t.reshape(1, s_t.shape[0]))
            if j%15==0:		#lower frequency noise
                noise_t[0][0] = train_indicator * max(epsilon, 0) * OU.function(a_t_original[0][0],  0.0 , 0.60, 0.50)  *spl
            a_t[0][0] = min(1, max(-1. (a_t_original[0][0] + noise_t[0][0])))


            s_t1, r_t, done, a_t[0], spl, colsens, info = env.step(a_t[0], j, i, s_t, automated = False)
          

            buff.add(s_t, a_t0[0], r_t, s_t1, done)      #Add replay buffer

            #symmetrical state space and actions - not for pedestrian use
            #s_t_sym = np.hstack((-s_t[0],- s_t[1],s_t[2],-s_t[3],-s_t[4],s_t[5],-s_t[6], s_t[7], -s_t[8]))
            #s_t1_sym = np.hstack((-s_t1[0],- s_t1[1],s_t1[2],-s_t1[3],-s_t1[4],s_t1[5],-s_t1[6], s_t[7], -s_t[8]))
            #a_t_sym = np.zeros([1,action_dim])

            #a_t_sym[0][0] = -a_t[0][0]
            #buff.add(s_t_sym, a_t_sym[0], r_t, s_t1_sym, done)      #Add replay buffer
            
            a_t0 = a_t


            #Do the batch update
            batch = buff.getBatch(BATCH_SIZE)
            states = np.asarray([e[0] for e in batch])
            actions = np.asarray([e[1] for e in batch])
            rewards = np.asarray([e[2] for e in batch])
            new_states = np.asarray([e[3] for e in batch])
            dones = np.asarray([e[4] for e in batch])
            y_t = np.asarray([e[1] for e in batch])

            for e in range(0, len(actions)):	#add noises (for td3)
                actions[e][0] =  max(-1, min((1, actions[e][0] + train_indicator * max(epsilon, 0) * OU.function(actions[e][0],  0.0 , 0.10, 0.10)))

            target_q_values1 = critic1.target_model.predict([new_states, actor.target_model.predict(new_states)])  
            target_q_values2 = critic2.target_model.predict([new_states, actor.target_model.predict(new_states)])
            target_q_values = target_q_values1
            for e in range(0, len(target_q_values1)):  
                target_q_values[e] = min(target_q_values1[e], target_q_values2[e])
            for k in range(len(batch)):
                if dones[k]:
                    y_t[k] = rewards[k]
                else:
                    y_t[k] = rewards[k] + GAMMA*target_q_values[k]
       
            if (train_indicator):
                loss1 += critic1.model.train_on_batch([states,actions], y_t) 
                loss2 += critic2.model.train_on_batch([states,actions], y_t) 
                loss = loss1 + loss2
                if j%2==0:	#update the NN in every two steps (td3)
                    a_for_grad = actor.model.predict(states)
                    grads = critic1.gradients(states, a_for_grad)
                    actor.train(states, grads)
                    actor.target_train()
                    critic1.target_train()
                    critic2.target_train()
    
            total_reward += r_t
            s_t = s_t1
        
            print("Episode", i, "Step", step, "Action", a_t, "Reward", r_t, "Loss", loss)
        
            step += 1
            if done:
                break

        if np.mod(i, 3) == 0:
            if (train_indicator):
                actorfile = "weights/"  + "_" + str(i) + "actormodel.h5"
                actorfile2 = "weights/"  + "_" +str(i) + "actormodel.json"
                criticfile1 = "weights/"  + "_" +str(i) + "criticmodel1.h5"
                criticfile21 = "weights/"  + "_" + str(i) + "criticmodel1.json"
                criticfile2 = "weights/" + "_" +str(i) + "criticmodel2.h5"
                criticfile22 = "weights/"  + "_" + str(i) + "criticmodel2.json"
                print("Now we save model")


                actor.model.save_weights(actorfile, overwrite=True)
                with open(actorfile2, "w") as outfile:
                    json.dump(actor.model.to_json(), outfile)

                critic1.model.save_weights(criticfile1, overwrite=True)
                with open(criticfile21, "w") as outfile:
                    json.dump(critic1.model.to_json(), outfile)

                critic2.model.save_weights(criticfile2, overwrite=True)
                with open(criticfile22, "w") as outfile:
                    json.dump(critic2.model.to_json(), outfile)

                actor.model.save_weights(actor_1, overwrite=True)
                with open(actor_2, "w") as outfile:
                    json.dump(actor.model.to_json(), outfile)

                critic1.model.save_weights(critic_11, overwrite=True)
                with open(critic_21, "w") as outfile:
                    json.dump(critic1.model.to_json(), outfile)

                critic2.model.save_weights(critic_12, overwrite=True)
                with open(critic_22, "w") as outfile:
                    json.dump(critic2.model.to_json(), outfile)

        print("TOTAL REWARD @ " + str(i) +"-th Episode  : Reward " + str(total_reward))

        print("Total Step: " + str(step))
        time.sleep(2)
        with open("episode_results.txt", "a") as myfile:
            myfile.write("TOTAL REWARD @ " + str(i) +"-th Episode  : Reward " + str(total_reward) + " STEPS: " + str(step-prev_step)+ " " + colsens  +"\n" 	)
        prev_step = step

        print("")

    print("Finish.")
    env.kill()

    



if __name__ == '__main__':

    main()
