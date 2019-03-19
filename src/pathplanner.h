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
#define FRONT_DIS_OWN          30.0
#define SIDE_DIS               15.0
#define TTC_THRESHOLD_FRONT    1.5
#define TTC_THRESHOLD_SIDE     3.0
#define DELTA                  0.00001

/*target lane change*/
#define TARGET_LEFT       -1
#define TARGET_RIGHT      1    

/*Lane change state*/
#define STATE_START       0
#define STATE_FINISH      1

using std::string;
using std::vector;
using namespace std;
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
     
                
            /*if(    (check_car_s >= car_s               )
                && ((check_car_s-car_s) < FRONT_DIS_OWN) 
                && (check_car_speed < car_speed        ) )
            {
                return true;
            }*/
        }
    }
    return false;
}

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

#endif