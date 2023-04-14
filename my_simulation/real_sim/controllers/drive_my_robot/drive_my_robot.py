"""drive_my_robot controller."""

#creating imports to use
from controller import Robot
from controller import DistanceSensor
from controller import Radar
from controller import RadarTarget
from controller import RangeFinder
from controller import LightSensor


def avoidance_movement():
    #just using regular speeed to adjust later
    SPEED = 4
   #getting snapshot image from rangefinder to then use
    image = RangeFinder.getRangeImage(range_finder)
    
    
    #just keeping regular distance placeholder values to change later
    left_distance = 100
    mid_distance = 100
    right_distance = 100
    
    #just using image taken earlier in order to get the respective
    #distance to the left middle and right of a given vehicle
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
    
    #setting motor values depending on distances seen for appropriate obstacle avoidance
    #if obsacle avoidance is needed
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
    
    
    range_finder = robot.getDevice('range_finder')
    #instantiating light sensor, creating a timestep of 64 and enabling the lightsensor
    light_sensor_left = robot.getDevice('light_sensor_left')
    light_sensor_right = robot.getDevice('light_sensor_right')
    
    timestep = 32
    #DistanceSensor.enable(sensor_left, timestep)
    #DistanceSensor.enable(sensor_right, timestep)
    #Radar.enable(radar_sensor, timestep)
    RangeFinder.enable(range_finder,timestep)
    LightSensor.enable(light_sensor_left,timestep)
    LightSensor.enable(light_sensor_right,timestep)

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
    
    # Main loop:
    while robot.step(timestep) != -1:
        light_value_left = LightSensor.getValue(light_sensor_left)
        light_value_right = LightSensor.getValue(light_sensor_right)
        print('this is left sensor: ' + str(light_value_left))
        print('this is right sensor: ' + str(light_value_right))
        
       
        #code used to perform "move to goal" behaviour of which
        #switch between move to goal or obstacle avoidance if the light intensity of the 
        #goal is higher than 280
        if (light_value_left < 280 and light_value_right < 280):
            avoidance_movement()
        else:
             if (light_value_left > 330 or light_value_right > 330):
                 right_motor.setVelocity(0.0)
                 left_motor.setVelocity(0.0)
             else:
                 if (light_value_left > light_value_right):
                    right_motor.setVelocity(5.0)
                    left_motor.setVelocity(0.0)
                 elif (light_value_left < light_value_right) :
                    left_motor.setVelocity(5.0)
                    right_motor.setVelocity(0.0)
                 elif (light_value_left == light_value_right):
                    left_motor.setVelocity(5.0)
                    right_motor.setVelocity(5.0)
        

    # Enter here exit cleanup code.
    