
#include <iostream>
#include <thread>
#include <math.h>
#include <unistd.h>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;

float res = 0.25;
float x_offset = 26 / 0.25;
float y_offset = 26 / 0.25;
float x_res = 52 / 0.25;
float y_res = 52 / 0.25;

int grid[208][208];


// Assumptions :
// 1) Line is drawn from left to right.
// 2) x1 < x2 and y1 < y2
// 3) Slope of the line is between 0 and 1.
//    We draw a line from lower left to upper
//    right.
// function for line generation
void bresenham(int x1, int y1, int x2, int y2, int tgt_x, int tgt_y)
{
   int m_new = 2 * (y2 - y1);
   int slope_error_new = m_new - (x2 - x1);
   for (int x = x1, y = y1; x <= x2; x++)
   {
      if (x == tgt_x && y == tgt_y && tgt_x != -1 && tgt_y != -1) {
        grid[x][y] += 1;
        cout << "(" << x << "," << y << ")" << endl;
      } 
      
      //else {
      //  grid[x][y] -= 1;
     //}

      // Add slope to increment angle formed
      slope_error_new += m_new;

      // Slope error reached limit, time to
      // increment y and update slope error.
      if (slope_error_new >= 0)
      {
         y++;
         slope_error_new  -= 2 * (x2 - x1);
      }
   }
}

void
callback(Robot* robot)
{
    //cout << "\n===" << endl;
    for (auto hit : robot->ranges) {
        int x = robot->pos_x;
        int y = robot->pos_y;
        if (hit.range <= 2) {

            //viz_hit(hit.range, hit.angle, x, y);

            // get hit info for the occupancy grid

            int angle = hit.angle + (M_PI / 2.0);
            float dx = 0.5 * hit.range * cos(angle);
            float dy = 0.5 * hit.range * sin(angle);
            float abs_x = x + dx;
            float abs_y = y + dy;
            abs_x = abs_x / res;
            abs_y = abs_y / res;
            float adj_x = abs(abs_x - x_offset);
            float adj_y = abs(abs_y - y_offset);
            adj_x = ceil(adj_x);
            adj_y = ceil(adj_y);

            float start_x = abs(x/res - x_offset);
            float start_y = abs(y/res - y_offset);
            start_x = ceil(start_x);
            start_y = ceil(start_y);

            if (start_x > adj_x && start_y > adj_y) {
                bresenham(adj_x, adj_y, start_x, start_y, adj_x, adj_y);    
            } else {
                bresenham(start_x, start_y, adj_x, adj_y, adj_x, adj_y);
            }

        } else {
            // update all estimated points based on misses
            float angles[7] = {2.35, 1.57, 0.725, 0, -0.785, -1.57, -2.356};
            for (float hit_angle : angles) {
                int angle = hit_angle + (M_PI / 2.0);
                float dx = 0.5 * 2.1 * cos(angle);
                float dy = 0.5 * 2.1 * sin(angle);
                int abs_x = round(x + dx);
                int abs_y = round(y + dy);
                abs_x = abs_x / res;
                abs_y = abs_y / res;
                int adj_x = abs(abs_x - x_offset);
                int adj_y = abs(abs_y - y_offset);

                int start_x = abs(x/res - x_offset);
                int start_y = abs(y/res - y_offset);
            
                if (start_x > adj_x && start_y > adj_y) {
                    bresenham(adj_x, adj_y, start_x, start_y, -1, -1);    
                } else {
                    bresenham(start_x, start_y, adj_x, adj_y, -1, -1);
                }   
            }
        }
        //cout << hit.range << "@" << hit.angle << endl;
    }

    if (robot->ranges.size() < 5) {
        return;
    }

    float lft = clamp(0.0, robot->ranges[2].range, 2.0);
    float fwd = clamp(0.0, robot->ranges[3].range, 2.0);
    float rgt = clamp(0.0, robot->ranges[4].range, 2.0);

    /*
    cout << "lft,fwd,rgt = "
         << lft << ","
         << fwd << ","
         << rgt << endl;
    */

    float spd = fwd - 1.0;
    float trn = clamp(-1.0, lft - rgt, 1.0);

    if (fwd < 1.2) {
      spd = 0;
      trn = 1;
    }

    /*
    cout << "spd,trn = " << spd << "," << trn << endl;
    */
    robot->set_vel(spd + trn, spd - trn);

    /*
    cout << "x,y,t = "
         << robot->pos_x << ","
         << robot->pos_y << ","
         << robot->pos_t << endl;
    */

    /*
    robot->set_vel(robot->pos_t, -robot->pos_t);
    */
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

void
draw_thread() 
{
    clear();
    while (true) {
        std::this_thread::sleep_for (std::chrono::seconds(1));
        //cout << "Drawing grid" << endl;
        for (int i = 0; i < x_res; i++) {
            for (int j = 0; j < y_res; j++) {
                if (grid[i][j] > 1) {
                    //cout << "Drawn: " << i << ", " << j << endl;
                    draw_index(j, i);
                }
            }
        }
    }
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread dthr(draw_thread);

    return viz_run(argc, argv);
}
