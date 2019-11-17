import glob
import os
import sys
import numpy as np
import math
from scipy.spatial import distance

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    print("eeee")
except IndexError:
    print("fffffffffff")
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


import time




class CarlaEnv(object):

    def __init__(self):

        self.actor_list = []
        self.client = carla.Client('localhost', 3001)
        self.world = None
        self.blueprint_library = None
        self.vehicle_bp = None
        self.vehicle = None
        self.camera_bp = None
        self.camera = None
        self.ped_bp = None
        self.pedestrian1 = None
        self.ped1_c = None
        self.control2 = None
        self.current_map = None
        self.lane_id = None
        self.lane_width = None
        self.spectator = None
        self.col_sensor = None
        self.collosion_with_ped = False
        self.ped_in_sight = 0
        self.fail_counter = 0
        self.uzenet = None
        self.spawn_point = None 
        self.initial_distance = 0
        self.initial_vehicle_speed = 0
        self.initial_speed = 0
        self.aaaa = None

    def spect_cam(self, vehicle):
        self.spectator = self.world.get_spectator()
        tr = vehicle.get_transform()
        tr.location.z+=12
        wp = self.current_map.get_waypoint(vehicle.get_transform().location,project_to_road=True, lane_type=carla.LaneType.Driving)
        tr.rotation = carla.Rotation(pitch=-90.000000, yaw=-180.289116, roll=0.000000)
        self.spectator.set_transform(tr)

    def start2(self):
        if (True):
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
            self.client = carla.Client('localhost', 3001)
            self.client.set_timeout(20.0)  
            self.client.load_world( '/Game/Carla/Maps/Town02')     
            self.world = self.client.get_world()
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.02
            self.world.apply_settings(settings)
            weather = carla.WeatherParameters(cloudyness = 20)
            self.world.set_weather(weather)
            self.current_map = self.world.get_map()
            self.blueprint_library = self.world.get_blueprint_library()
            self.spawn_point = carla.Transform(carla.Location(x=173.758665, y=105.552750, z=0.270000), carla.Rotation(pitch=0.000000, yaw=-180.289116, roll=0.000000))


    def kill(self):

        print('destroying actors')
        for actor in self.actor_list:
            actor.destroy()
        print('done.')

    def collevent(self, event):

        eventframe = event.frame
        eventactor = event.actor
        eventotheractor = event.other_actor
        if eventotheractor.id == self.pedestrian1.id:
                print("COLLISION!!!!!!!!!!!")
                self.collosion_with_ped = True
        print (eventframe, "      ", eventactor, "       ", eventotheractor)


    def setup_car(self):		#AUTÓ LÉTREHOZÁSA
        """
        Spawns actor-vehicle to be controled.
        """
        self.collosion_with_ped = False
        self.vehicle_bp = self.blueprint_library.find('vehicle.audi.etron')
        self.vehicle_bp.set_attribute('color', '255,105,180')
        loc = carla.Location(65.568863, 4.308813, 1.843102)
        rot = carla.Rotation(0,0.8,0)
        transform = carla.Transform(loc, rot)

        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.spawn_point)
        #self.vehicle = self.world.spawn_actor(self.vehicle_bp, random.choice(self.world.get_map().get_spawn_points()))
#------------------------------------------------------
# collision sensor 
#------------------------------------------
        col_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        self.col_sensor = self.world.spawn_actor(col_bp, carla.Transform(), attach_to=self.vehicle)
        self.col_sensor.listen(lambda event: self.collevent(event))
#---------------------------------------------------------------
        self.vehicle.set_autopilot(False)
        self.actor_list.append(self.vehicle)
        self.lane_width = self.current_map.get_waypoint(self.spawn_point.location).lane_width
        self.lane_id = self.current_map.get_waypoint(self.spawn_point.location,project_to_road=True, lane_type=carla.LaneType.Driving).lane_id
        self.initial_vehicle_speed = 10 + random.random()*5
        self.ped_in_sight = 0
        self.uzenet = 'AVOIDED PEDESTRIAN      '


        print('created %s' % self.vehicle.type_id)

    def setup_pedestrian(self):		#GYALOGOS LÉTREHOZÁSA
        ped_suc = False
        while ped_suc == False:
            try: 
                ped_suc = True
                self.ped_bp = random.choice(self.blueprint_library.filter('walker'))
                if True: #self.ped_in_sight == 1: #move pedestrian if in sight
                    self.pedestrian1 = self.world.spawn_actor(self.ped_bp, carla.Transform(carla.Location(120, 105.5,1), carla.Rotation(0,90,0)))
                else:
                    self.pedestrian1 = self.world.spawn_actor(self.ped_bp, random.choice(self.world.get_map().get_spawn_points()))
                #self.walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                #self.ped1_c = self.world.spawn_actor(self.walker_controller_bp, carla.Transform(carla.Location(-30, 8,1), carla.Rotation(0,-90,0)),self.pedestrian1)
                self.actor_list.append(self.pedestrian1)


                #self.pedestrian1.set_max_speed(8)
                self.initial_distance = random.random()*10 + 8
                self.initial_speed = min(6, (0.5+ 4*random.random() ) *self.initial_vehicle_speed/self.initial_distance) #random.random()*2+0.5
                self.control2 = carla.WalkerControl(carla.Vector3D(0, 1, 0), self.initial_speed, False)
                #self.pedestrian1.apply_control(self.control2)
            except:

                print("FAILED TO MOVE PEDESTRIAN")
                ped_suc = False
            
    
    def braking_distance(self, v, egyperd):		#NOT USED HERE
        return (1/egyperd)/ (v**2/16)

    def ped_sensor(self, vehicle_tf, ped_tf):	#PEDESTRIAN DISTANCE AND ANGLE

        p_veh = (vehicle_tf.location.x+2.5*math.cos(vehicle_tf.rotation.yaw), vehicle_tf.location.y+2.5*math.sin(vehicle_tf.rotation.yaw))
        p_ped = (ped_tf.location.x, ped_tf.location.y)
        d_ped_veh = distance.euclidean(p_veh, p_ped)
        vector1 = np.array([math.cos(vehicle_tf.rotation.yaw*math.pi/180), math.sin(vehicle_tf.rotation.yaw*math.pi/180)])
        vector2 = np.subtract( p_ped,p_veh)
        dotproduct = np.dot(vector1, vector2)

        angle = np.arccos(np.sign(dotproduct) * dotproduct / math.sqrt(vector2[0]*vector2[0] + vector2[1] * vector2[1]) )
        d_ped_veh_reciprok = 0
        if d_ped_veh==0:
            d_ped_veh = 0.001

        d_ped_veh_reciprok = 1/d_ped_veh
        self.fail_counter =0
        return d_ped_veh_reciprok, angle
        
        



    def snap_to_s_t(self, world_snapshot, currentmap, vehicle, ped):	#OB FÜGGVÉNY
        vehicle_snapshot = world_snapshot.find(vehicle.id)
        ped_snapshot = world_snapshot.find(ped.id)
        vehicle_transform = vehicle_snapshot.get_transform()
        vehicle_velocity = vehicle_snapshot.get_velocity()
        vehicle_ang_vel = vehicle_snapshot.get_angular_velocity()
        vehicle_acceleration = vehicle_snapshot.get_acceleration()

        ped_transform = ped_snapshot.get_transform()
        
        d_ped_veh, angle_ped_veh = self.ped_sensor(vehicle_transform, ped_transform)


        lane_offset_x = 0
        lane_offset_y = 0
        i = 1

        
        road_waypoint = None
        road_waypoint = currentmap.get_waypoint(vehicle_transform.location,project_to_road=True, lane_type=carla.LaneType.Driving)


        while True:
            temp_loc = carla.Location(vehicle_transform.location.x+lane_offset_x ,vehicle_transform.location.y+lane_offset_y,vehicle_transform.location.z)
            road_waypoint = currentmap.get_waypoint(temp_loc,project_to_road=True, lane_type=carla.LaneType.Driving)
            l_id = road_waypoint.lane_id

            if l_id == self.lane_id:  
                break
            #lane_offset_x = np.power(-1, i)*i/2#*#math.cos(road_waypoint_old.transform.rotation.yaw*math.pi/180)
            lane_offset_y =np.power(-1, i)*i#*math.sin(road_waypoint_old.transform.rotation.yaw*math.pi/180)

            i+=1
            if i==100:
                print("i>100")
            if i==200:    
                break
        p_veh = (vehicle_transform.location.x, vehicle_transform.location.y)
        p_road = (road_waypoint.transform.location.x, road_waypoint.transform.location.y)

        v_veh_road = (p_veh[0] - p_road[0], p_veh[1] - p_road[1])
        v_road = (math.cos(road_waypoint.transform.rotation.yaw*math.pi/180), math.sin(road_waypoint.transform.rotation.yaw*math.pi/180))
        lat_err = -distance.euclidean(p_veh, p_road)*np.sign(np.cross(v_veh_road, v_road))

        ang_err = vehicle_transform.rotation.yaw - road_waypoint.transform.rotation.yaw
        if ang_err > 180:
            ang_err = -360+ang_err
        if ang_err < -180:
            ang_err = 360+ang_err
        speed = math.sqrt(vehicle_velocity.x**2 +  vehicle_velocity.y**2)
        acc = math.sqrt(vehicle_acceleration.x**2 + vehicle_acceleration.y**2)
        speedX = abs(speed*math.cos(vehicle_transform.rotation.yaw*math.pi/180-math.atan2(vehicle_velocity.y, vehicle_velocity.x)))
        speedY = speed*math.sin(vehicle_transform.rotation.yaw*math.pi/180-math.atan2(vehicle_velocity.y, vehicle_velocity.x))
        ang_vel = vehicle_ang_vel.z
        accX = acc*math.cos(vehicle_transform.rotation.yaw*math.pi/180-math.atan2(vehicle_acceleration.y, vehicle_acceleration.x))
        accY = acc*math.sin(vehicle_transform.rotation.yaw*math.pi/180-math.atan2(vehicle_acceleration.y, vehicle_acceleration.x))
        return np.hstack((lat_err, ang_err, speedX, speedY, ang_vel, accX, accY, d_ped_veh, angle_ped_veh ))




    def automated_drive(self, world_snapshot, brakes=False, avoid=False):
        ob= self.snap_to_s_t(world_snapshot, self.current_map, self.vehicle, self.pedestrian1)
        accel = 0
        target_speed = self.initial_vehicle_speed
        target_oldaliranyu = 0.3
        oldaliranyu_sebhiba = 1.11 * target_oldaliranyu - abs(ob[3])
        sebhiba = 1.01 * target_speed - ob[2]
        if brakes:
            if abs(ob[3]) <0.3  or ob[2] < 15:
                accel = min(1, 1* math.pow(sebhiba,3))
            else:
                accel = max(-1, math.pow(oldaliranyu_sebhiba - 1, 3))
        else:
            target_speed = 20
            sebhiba = 1.01 * target_speed - ob[2]
            accel = min(1, math.pow(sebhiba, 3))
        if ob[2] == 0:
            ob[2] = 0.05
        steering_angle = -ob[1]*math.pi/180 - math.atan2(2*(  ob[0]), ob[2])
        if avoid:
            steering_angle = -ob[1]*math.pi/180 - math.atan2(2*(  ob[0]+4), ob[2])
        steering_jel = min(1, max(-1, 2*steering_angle))
        control = carla.VehicleControl()
        if accel>0:
            control.throttle = accel
            control.brake = 0
        else:
            control.throttle = 0
            control.brake = accel
        control.steer = steering_jel
        if ob[2] < 15:
            control.gear = 1
        elif ob[2] < 30:
            control.gear = 2
        else:
            control.gear = 2
        control.manual_gear_shift = True

        return control

    def throttlecontrol(self, speed):
        accel = 0
        target_speed = self.initial_vehicle_speed
        sebhiba = 1.01 * target_speed - speed
        accel = min(1, 1* math.pow(sebhiba,3))
        return accel
        

    def step(self, u, episode_step, ep, st, automated=False):		
        a = self.world.tick()
        at= u
        done = False
        self.aaaa = True
        control = carla.VehicleControl()
        self.spect_cam(self.vehicle)
        world_snapshot = self.world.get_snapshot()
        ob= self.snap_to_s_t(world_snapshot, self.current_map, self.vehicle, self.pedestrian1)
        if 1/ob[7] < self.initial_distance:
            self.ped_in_sight = 1
        """if self.ped_in_sight == 0:
            ob[7] = 0
            ob[8] = 0
        else:
            while self.aaaa == True:
                if self.fail_counter > 50:
                    done = True
                    print("CAN'T MOVE PEDESTRIAN")
                    break
                try:
                    self.pedestrian1.apply_control(self.control2)
                    self.aaaa = False

                except:
                    self.aaaa = True
                    print("I FAILED TO MOVE IT MOVE IT")
                    self.fail_counter +=1
         """
        if automated:
            control = self.automated_drive(world_snapshot)
        else:
            at = u
            avoiid = False
            if ob[7] >0.1 and world_snapshot.find(self.vehicle.id).get_transform().location.x >128 	:
                avoiid =True
            if ep%5 == 1 :	#every 5th episode is automated
                control = self.automated_drive(world_snapshot, brakes = False, avoid=avoiid)
            else:
                accel = self.throttlecontrol(ob[2])                        
                control.manual_gear_shift = True
                if accel  > 0:
                    control.throttle = accel
                    control.brake = 0
                else:
                    control.throttle = 0
                    control.brake = accel
                control.steer = min(1, max(-1,u[0]))
                if ob[2] < 15:
                    control.gear = 1
                elif ob[2] < 30:
                    control.gear = 2
                else:
                    control.gear = 3
        at[0] = control.steer
        self.vehicle.apply_control(control)


        #-----------REWARDING---------------------------------------
        #-----------------------------------------------------------

        koztes_cucc = ob[1]/180*math.pi + math.atan2(5*ob[0], ob[2]/3.6) + math.atan2( ob[4]/180*math.pi * 2.6, ob[2]/3.6)     
   
        #reward = -1.00/ob[7] + self.lane_width/2 - abs(ob[0])
        #reward =  self.lane_width/2 - abs(ob[0])
        #progress = ob[2]*np.cos(ob[1]*math.pi/180) -np.abs(ob[2]*np.sin(ob[1]*math.pi/180)) - ob[2] * np.abs(ob[0]*math.pi/180) - abs(ob[4] - st[4])

        #THESE ARE SOME ALTERNATIVE REWARDS
        #reward =  np.tanh(sp/150) * max(-1, min(np.cos(koztes_cucc *2 /3),-np.abs(4/3*koztes_cucc/math.pi)+1))  
        #reward =  np.tanh(sp/150) * np.cos(math.sqrt(np.abs(2*koztes_cucc*math.pi)))
        #koztes2 = 1-2* np.exp(-2*self.braking_distance(ob[2], ob[7]))
        #reward = progress
        #koztes3 = np.tanh(1/(ob[7]+0.0001)+4)*np.sign(world_snapshot.find(self.vehicle.id).get_transform().location.x -120)

        koztes3 = np.tanh(world_snapshot.find(self.vehicle.id).get_transform().location.x -128) *np.sign(world_snapshot.find(self.vehicle.id).get_transform().location.x -120)
        #reward = (1 - abs(ob[0]/self.lane_width))*koztes3
        reward = np.cos(math.sqrt(np.abs(koztes_cucc*math.pi*2 /3)))*koztes3
        #(1-2*np.exp( -2*( 1/((0.0001 + ob[7])*(0.0001 +ob[2])**2)/16))) #-  2*ob[7]

        if abs(ob[0]) > self.lane_width*3:
            done = True
            reward = -10
        if episode_step>100 and ob[2] < 2:
            done = True
            reward = -10
            self.uzenet = '!!!STOPPED!!!'
            if world_snapshot.find(self.vehicle.id).get_transform().location.x < 120:
                reward = 10
                self.uzenet = "STOPPED, BUT PASSED!"
        if abs(ob[1]) > 90:
            done = True
            reward = -10
            self.uzenet = '!!!TURNED BACK!!!'
        if self.collosion_with_ped:
            done = True
            self.uzenet = '!!!COLLISION!!!'
            reward = -100

        if world_snapshot.find(self.vehicle.id).get_transform().location.x < 110:
            done = True
            reward = 100*math.cos(ob[1]/180*math.pi)*(-0.008130825 + (0.9762081 - (-0.008130825))/(1 + (abs(ob[0])/1.32221)**3.15538))	#cos(angle)*sigmoid(lateral_error)
            self.uzenet += str(reward)
            
            
        return ob, reward, done, at, self.ped_in_sight,self.uzenet, {}

        

    def destroy(self):

        print('destroying actors')
        for actor in self.actor_list:
            actor.destroy()
        print('done.')


