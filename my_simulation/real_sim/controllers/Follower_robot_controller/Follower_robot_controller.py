"""drive_my_robot controller."""

#creating imports to use
from controller import Robot
from controller import DistanceSensor
from controller import Radar
from controller import RadarTarget
from controller import RangeFinder
from controller import LightSensor

#creating method for main obstacle avoidance method 
def avoidance_movement():
    SPEED = 4
    
    #getting snapshot image and number of targets for on radar sensor
    #in order to determine whether or not we need to follow the leader
    #or avoid the obstacle later on
    image = RangeFinder.getRangeImage(range_finder)
    num_targets = Radar.getNumberOfTargets(radar_sensor)
    target = Radar.getTargets(radar_sensor)
    
    
    left_distance = 100
    mid_distance = 100
    right_distance = 100
    
    #using a for loop to interate through every pixel on a single dimensino of the
    #range finder in order to divide and associate each pixel intensity to a given
    #distance parameter
    for i in range(0,range_finder_width):
        distance = RangeFinder.rangeImageGetDepth(image,range_finder_width,i,40) 
        #print('this is current distance: ' + str(distance) + ' for i = ' + str(i))
        
        if i < range_finder_width/3:
            if distance <left_distance:
                left_distance = distance
        elif i >= 2*(range_finder_width/3):
            if distance < right_distance:
                right_distance = distance
        else :
            if distance < mid_distance:
                mid_distance = distance
    
    #setting motor values depending on distances seen
    if mid_distance <= 0.05 and right_distance <= 0.05 and left_distance < 0.05:
        left_speed = SPEED - left_distance
        right_speed = SPEED - right_distance
    elif right_distance < 0.08:
        left_speed = SPEED - left_distance
        right_speed = -(SPEED - right_distance)
    elif left_distance <= 0.08:
        left_speed = -(SPEED - left_distance)
        right_speed = SPEED
    elif mid_distance > 0.06 and right_distance > 0.05 and left_distance > 0.06:
        left_speed = -(SPEED - left_distance)
        right_speed = -(SPEED - right_distance)
        
        #This portion of the code determines the motor output depending on
        #whether or not a target has been detected
        #if there is a target the left and right motor speeds will be overwitten
        #from the ones calculated above for obstacle avoidance in order to perform
        #the "follow the leader" behaviour
        if num_targets > 0:
            distance = target[0].distance
 
            
            
            if distance > 0.025 and target[0].azimuth < 0.005:
                left_speed = (SPEED-3) + 5*target[0].azimuth
                right_speed = -(SPEED-3) + 5*target[0].azimuth
            elif distance > 0.025 and target[0].azimuth > 0.1:
                left_speed = -(SPEED-3) + 5*target[0].azimuth
                right_speed = (SPEED-3) + 5*target[0].azimuth
                
                 
     #second motor is left and main motor is rigtht
    right_motor.setVelocity(-right_speed)
    left_motor.setVelocity(-left_speed)
        
        
        
#creating main function in order to run code as a script
if __name__ == "__main__":
    # create the Robot instance in order to use robot.
    robot = Robot()
    #instantiate distance sensor to receieve calculations
    #sensor_left = robot.getDevice('distance_sensor1')
    #sensor_right = robot.getDevice('distance_sensor2')
    
    light_sensor = robot.getDevice('light_sensor')
    range_finder = robot.getDevice('range_finder')
    radar_sensor = robot.getDevice('radar_sensor')
    #instantiating light sensor, creating a timestep of 64 and enabling the lightsensor
    SPEED = 4
    timestep = 32
    time = -3
    
    
    RangeFinder.enable(range_finder,timestep)
    Radar.enable(radar_sensor,timestep)
    LightSensor.enable(light_sensor,timestep)
    
    right_motor = robot.getDevice('rightmotor')
    left_motor = robot.getDevice('leftmotor')
    max_speed = 5
    
    #setting motor position and velocity
    right_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    left_motor.setPosition(float('inf'))
    
    
    #using variable to switch between first and second transfer function to use
    #during testing phase of simulation
    transfer_function = 0
    count = 0
    
    range_finder_width = RangeFinder.getWidth(range_finder)
    range_finder_height = RangeFinder.getHeight(range_finder)
    range_finder_last_seen_angle = 0
    count = 0
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        ##perform regular avoidance movement unless lightlevel on sensor
        ##reaches above 550 to indicate robot has reached appropriate destination
        
        num_targets = Radar.getNumberOfTargets(radar_sensor)
        target_leader = Radar.getTargets(radar_sensor)
        light_value = LightSensor.getValue(light_sensor)
        
        #This first portion of the code determines if the goal has been detected
        #if there is, the vehicle will come to a complete stop which is the "Stop at Goal"
        #behaviour described in the report
        
        if (light_value >= 300):
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
        else:
        
            #IF there is no goal in site, then we can proceed with determineing if we need
            #follow the leader, find the leader ("remembrance bahaviour") or avoid obstacles 
            #want to keep track of the current angle the target was last seen if there was
            # a target in the first place
            if num_targets == 1:
                range_finder_last_seen_angle = target_leader[0].azimuth
            
            #This portion first checks if the target it even in site, if not then we need to perform either rememberance or 
            #random movemnet until we hit the target
            if num_targets == 0:
                #count number goes up by 1 ever 0.032 seconds
                #therefore to wait 4 seconds we need to count for ~135 or more until
                #we find the target
                count = count+1
                print('now counting' + str(count))
                #first check if 
                if count >= 135 and count < 340:
                    
                    #print('this is regular target finder movement')
                    #trying to move follower based on angle last seen to try and
                    #give some form of memory to the follower robot
                    if range_finder_last_seen_angle > 0: 
                        #This means that the target was last seen to the right
                        #of the robot, therefore lets turn the robot to the right
                        left_speed = -(SPEED) + range_finder_last_seen_angle
                        right_speed = 0.0
                        left_motor.setVelocity(right_speed)
                        right_motor.setVelocity(left_speed)
                    elif range_finder_last_seen_angle < 0:
                        #this means that the target was last seen on the left
                        #thus move right motor to get robot looking to the left
                        left_speed =  0.0
                        right_speed = -(SPEED) + range_finder_last_seen_angle
                        left_motor.setVelocity(right_speed)
                        right_motor.setVelocity(left_speed)
                else:
                    #If the taget IS IN SITE, then we need to determine if we need to avoid an obstacle or follow the leader
                    avoidance_movement()
                    
            elif num_targets == 1:
                #reset the count if the target has been found
                print('just reset the count')
                count = 0
                avoidance_movement()


    # Enter here exit cleanup code.
    