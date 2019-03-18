#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <string>
#include <vector>

/* Lanes define*/
#define LEFT_LANE         0
#define MID_LANE          1
#define RIGHT_LANE        2

/*define safty distance for own lane front, side lane front and rear*/ 
#define FRONT_DIS_OWN     30.0
#define FRONT_DIS_SIDE    50.0
#define REAR_DIS_SIDE     25.0
/*target lane change*/
#define TARGET_LEFT       -1
#define TARGET_RIGHT      1    


bool egoTooCloseFront(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size)
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
            double  check_speed_front = sqrt(vx*vx+vy*vy);
            double  check_car_s = sensor_fusion[i][5];

            check_car_s+= ((double)prev_size *.02*check_speed_front);
            if((check_car_s >= car_s)) {
                if((check_car_s-car_s) < distance_to_keep_front){
                    cout<<"too close"<<(check_car_s-car_s)<<" lane"<< lane<<endl;
                    return true;
                }
            }
        }
    }
    return false;
}

bool SideChangePossible(vector<vector<double>> sensor_fusion,int lane,double car_s,int prev_size, int target_lane)
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
                double  check_speed_front = sqrt(vx*vx+vy*vy);
                double  check_car_s = sensor_fusion[i][5];

                check_car_s+= ((double)prev_size *.02*check_speed_front);
                if(      (      (check_car_s >= car_s                 )
                            &&  ((check_car_s-car_s) <= FRONT_DIS_SIDE)  )
                     ||  (      (check_car_s <= car_s               )
                            &&  ((car_s-check_car_s) < REAR_DIS_SIDE)  )
                {
                  return false;      
                }
            }
        }
        return true;
    }
    return false;
}

int checklane(double d)
{
    int lane = 0;
    if (d >= 0 and d < 4)
    {
        lane = LEFT_LANE;
    }
    else if (d>=4 and d < 8)
    {
        lane  = MID_LANE;
    }
    else 
    {
        lane = RIGHT_LANE;
    }
    return lane;
}

