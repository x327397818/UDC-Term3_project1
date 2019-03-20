# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
This project simulates a virtual highway with numerous cars running at speeds +- 10 mph of a 50 mph speed limit.

The goal of was to program the "ego" car to:

* The car is able to drive at least 4.32 miles without incident.
* Stay close to the speed limit without exceeding it
* Accelerate and decelerate within jerk limits
* Avoid all collisions
* Drive inside the lane lines, except when changing lanes
* Able to change lane when there is open on side lanes

As shown in the [video](https://github.com/x327397818/UDC-Term3_project1/tree/master/Video), this implementation can run at least 10 miles / 11 minutes without incident and meet all points in the [rubric](https://review.udacity.com/#!/rubrics/1020/view).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Reflection

Header file [spline.h](https://github.com/x327397818/UDC-Term3_project1/blob/master/src/spline.h) has been added for smooth path calculation. There is also a header file [pathplanner.h](https://github.com/x327397818/UDC-Term3_project1/blob/master/src/pathplanner.h) for some prediction and desicion help function used in path planning.

The implementation is divided into three steps:

1. Prediction based on fusion data
2. Decision of the behavior
3. Generation of the vehicle's trajectory

### Prediction

'''

          int curr_lane = checklane(car_d);
          /*check if ego is possible to kick front car*/
          bool front_collision = egokicksFront(sensor_fusion,curr_lane,car_s,prev_size,car_speed);
		  
'''

In this part, check if ego is going to hit front car. `egokicksFront()` is using TTC(time to collision) to see how far ego is from collision on timewise.

'''


bool egokicksFront(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size,double car_speed)
{

    for(int i=0; i <sensor_fusion.size();i++)
    {

        float d = sensor_fusion[i][6];

        if(    d < (2+4*lane+2)
            && d > (2+4*lane-2)  )
        {
            int     id = sensor_fusion[i][0];
            double  vx = sensor_fusion[i][3];
            double  vy= sensor_fusion[i][4];
            double  check_car_speed = sqrt(vx*vx+vy*vy);
            double  check_car_s = sensor_fusion[i][5];
            double  temp_ttc = 999.99;
            
            check_car_s+= ((double)prev_size *.02*check_car_speed);
            
            if (abs(check_car_speed - car_speed) > DELTA)
            {
                temp_ttc = (check_car_s-car_s)/(car_speed - check_car_speed);  
            }
                
                
            if(temp_ttc <= TTC_THRESHOLD_FRONT && temp_ttc > 0 && (check_car_s >= car_s))
            {
                return true;      
            }
        }
    }
    return false;
}

'''

###Behavior
This part, based on the situation ego is facing. Make decision on the following behavior in following state
* Speed up
* Slow down
* Change to Left lane
* Change to right lane

The basic logic is,
* 1. Start to make decision when ego has finished last lane.
* 2. If ego is going to hit front car, check if it can change to left lane. 
* 3. If cannot change to left lane, check if it can change to right lane.
* 4. If cannot change to right lane either, start to slow down.
* 5. If no collision would happen and ego speed is lower than target speed, speed up.

'''

          /*set correct state*/
          lane_change_state = (curr_lane == lane)? STATE_FINISH:STATE_START;

          /*plan following behavior*/
          if(front_collision)
          {
            
            if(lane_change_state != STATE_START)
            {
                bool bchange_left = SideChangePossible(sensor_fusion,curr_lane,car_s,prev_size, TARGET_LEFT, car_speed);
           
                if(bchange_left)
                {
                    lane = lane + TARGET_LEFT;
                    lane_change_state = STATE_START;
                }
                else
                {
                    bool bchange_right = SideChangePossible(sensor_fusion,curr_lane,car_s,prev_size, TARGET_RIGHT, car_speed);
                    
                    if(bchange_right)
                    {
                      lane = lane + TARGET_RIGHT;
                      lane_change_state = STATE_START;
                    }
                    else
                    {
                        ref_vel -= .224;
                    }
                }
            }
          }
          else if(ref_vel < max_velocity)
          {
              ref_vel += .224;
          }
		  
'''

`SideChangePossible()` is using both TTC and distance to see if changing would have collision possibility.

'''

bool SideChangePossible(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size, int target_lane,double car_speed)
{
    int new_lane = lane+target_lane;

    if(     (new_lane >= 0)
        &&  (new_lane <= 2)  )
    {

        for(int i=0; i <sensor_fusion.size();i++)
        {

            float d = sensor_fusion[i][6];

            if(d < (2+4*new_lane+2) && d > (2+4*new_lane-2))
            {
                int     id = sensor_fusion[i][0];
                double  vx = sensor_fusion[i][3];
                double  vy= sensor_fusion[i][4];
                double  check_car_speed = sqrt(vx*vx+vy*vy);
                double  check_car_s = sensor_fusion[i][5];
                double  temp_ttc = 999.99;
                

                check_car_s+= ((double)prev_size *.02*check_car_speed);
                
                if (abs(check_car_speed - car_speed) > DELTA)
                {
                  temp_ttc = (check_car_s-car_s)/(car_speed - check_car_speed);  
                }
                
                cout<<"The dis is "<<(check_car_s-car_s)<<endl;
                
                if(  temp_ttc <= TTC_THRESHOLD_SIDE && temp_ttc > 0 || abs(check_car_s-car_s) < SIDE_DIS)
                {
                  return false;      
                }
            }
        }
        return true;
    }
    return false;
}


'''

###Trajectory

From line 164 to line 254. I am using the same logic as explaining in Q&A video to applying trajectory creation. Spline libary is used to ensure smooth moving.