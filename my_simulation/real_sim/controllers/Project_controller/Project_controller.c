#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <webots/motor.h>
#include <webots/range_finder.h>
#include <webots/robot.h>
#include <webots/utils/system.h>
#include <webots/nodes.h>
#include <webots/radar.h>
#include <webots/device.h>
//Include libraries for the devices we are using.

#define SPEED 4
#define TIME_STEP 32
//Define the speed of the robot and the simulation time.

int main() {
  WbDeviceTag range_finder, left_motor, right_motor, radar;
  int range_finder_width, range_finder_height;
  int left_speed, right_speed, i, j;
  float distance;
  bool tag = false;

  wb_robot_init();
  /*
   First we get a handler to the range-finder device and then we open and place
   it. We also store its height, width, near and far parameters for
   further use. Use the name from the scene tree to find the device.
   */
  range_finder = wb_robot_get_device("range finder");
  wb_range_finder_enable(range_finder, TIME_STEP);
  range_finder_width = wb_range_finder_get_width(range_finder);
  range_finder_height = wb_range_finder_get_height(range_finder);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("leftmotor");
  right_motor = wb_robot_get_device("rightmotor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* Handler for the radar node */
  radar = wb_robot_get_device("radar");
  wb_radar_enable(radar, TIME_STEP);
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    /* This is used to refresh the range-finder. */
    const float *image = wb_range_finder_get_range_image(range_finder);
    
    /* Get Radar data */
      int targets_number = wb_radar_get_number_of_targets(radar);
      const WbRadarTarget *targets = wb_radar_get_targets(radar);
      /*printf("%s see %d targets.\n", wb_robot_get_name(), targets_number);
      for (i = 0; i < targets_number; ++i)
        printf("---target %d: distance=%lf azimuth=%lf\n", i + 1, targets[i].distance, targets[i].azimuth);
    */
    
    /* Store distance values to find the smallest distance in each area. 
    This also serves as a reset everytime the while loop iterates*/
    float left_distance = 100;
    float mid_distance = 100;
    float right_distance = 100;

    /*
     Here we will go through each pixel in the image from the depth finder.
     To keep it simple we will only analyse one line (near the middle).
     We will determine which area the pixel is in, and therefore which distance
     we should compare it to.
     This can easily be expanded to iterate through all pixels by nesting another
     if statement.
     */
    for (i = 0; i < range_finder_width; i++) 
    {
      distance = wb_range_finder_image_get_depth(image, range_finder_width, i, 40);
    
      if(i < range_finder_width/3)
      {
        if(distance < left_distance)
        {
          left_distance = distance;
        }
      }
      else if (i >= 2*(range_finder_width/3))
      {
        if(distance < right_distance)
        {
          right_distance = distance;
        }
      }
      else
      {
        if(distance < mid_distance)
        {
          mid_distance = distance;
        }
      }
      
    }
    
    //Debug smallest distances
    //printf("left: %f mid: %f right: %f\n", left_distance, mid_distance, right_distance);
    /*
      Here we use the distances to determine where to move.
      If there is room in the middle, go forward.
      Else see where the closest wall is and turn away.
    */
    if (mid_distance <= 0.035 || right_distance <= 0.035 || left_distance < 0.035)
    {
      left_speed = SPEED;
      right_speed = -SPEED;
      //printf("reverse ");
    }
    else if (right_distance < 0.07)
    {
      left_speed = SPEED;
      right_speed = SPEED;
      //printf("left ");
    }
    else if (left_distance <= 0.07)
    {
      left_speed = -SPEED;
      right_speed = -SPEED;
      //printf("right ");
    }
    else if (mid_distance > 0.05 && right_distance > 0.06 && left_distance > 0.06)
    {
      left_speed = -SPEED;
      right_speed = SPEED;
      if(targets_number >= 1)
      {
        distance = 100; 
        for(i = 0; i < targets_number; i++)
        {
          if(distance > targets[i].distance) {distance = targets[i].distance; j = i;}
        }
        if (distance > 0.015)
        {
          left_speed = -(SPEED + 2) - 4*targets[j].azimuth;
          right_speed = (SPEED + 2) - 4*targets[j].azimuth;
          //printf("following target");
        }
      }
    }

    /* Set the motor speeds. */
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  // These are required
  wb_robot_cleanup();

  return 0;
}