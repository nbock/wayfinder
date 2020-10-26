#include <iostream>
#include <thread>
#include <math.h>
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <vector>
#include <bits/stdc++.h>

#include "robot.hh"
#include "viz.hh"

using std::cout;
using std::endl;

float res = 0.25;
float x_offset = 26 / res;
float y_offset = 26 / res;
float x_res = 52 / res;
float y_res = 52 / res;
float log_odd_occ = 0.05;
float log_odd_free = 0.95;
float count = 0.00;
float observed_x = 0.00;
float observed_y = 0.00;
float goal_x = abs( (20.0 / res) - x_offset);
float goal_y = abs( (0.0 / res) - y_offset);
int estimate_x;
int estimate_y;

#define ROW 208
#define COL 208


float grid[208][208];

std::mutex mtx;

// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef std::pair<double, std::pair<int, int>> pPair;

std::stack<Pair> curr_path;

// A structure to hold the neccesary parameters
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

// Attribution: Modified from Geeks for Geeks
std::vector<std::tuple<int,int>> bresen(int x1, int y1, int x2, int y2)
{
    int m_new = 2 * (x2 - x1);
    int slope_error_new = m_new - (y2 - y1);
    std::vector<std::tuple<int, int>> points;
    for (int x = x1, y = y1; y <= y2; y++)
    {
        // add to vector
        points.push_back(std::make_tuple(x, y));

        // Add slope to increment angle formed
        slope_error_new += m_new;

        // Slope error reached limit, time to
        // increment y and update slope error.
        if (slope_error_new >= 0)
        {
            x++;
            slope_error_new  -= 2 * (y2 - y1);
        }
    }

    return points;
}

float heuristic(int x1, int y1) {
    // straight line distance to the goal
    float raw = sqrt( pow((x1 - goal_x), 2) + pow((y1 - goal_y), 2) );

    // we know that each wall adds at least one square more we need to travel
    int wall = 0;
    std::vector<std::tuple<int,int>> points = bresen(x1, y1, goal_x, goal_y);
    for (int i = 0; i < points.size(); i++) {
        int row = std::get<0>(points[i]);
        int col = std::get<1>(points[i]);

        mtx.lock();
        float square = grid[row][col];
        mtx.unlock();

        if (square >= 0.5) {
            // wall hits add a lot of distance
            wall += 20;
        }
    }
    return raw + wall;
}

// A Utility Function to trace the path from the source
// to destination
std::stack<Pair> tracePath(cell cellDetails[][COL])
{
    //printf ("\nThe Path is ");
    int row = goal_x;
    int col = goal_y;

    std::stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {
        Path.push (std::make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (std::make_pair (row, col));

    return Path;
}

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < x_res) &&
           (col >= 0) && (col < y_res);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int row, int col)
{
    // Returns true if the cell is not blocked else false
    mtx.lock();
    float square = grid[row][col];
    mtx.unlock();

    if (square < 0.5) {
      return true;
    } else {
      //cout << "(" << row << ", " << col << "): " << "BLOCKED" << endl;
      return false;
    }
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col)
{
    if (row == goal_x && col == goal_y)
        return (true);
    else
        return (false);
}

// A start function for finding goal
// Adapted from https://www.geeksforgeeks.org/a-search-algorithm/
std::stack<Pair> aStarSearch(int x1, int y1) {

  std::stack<Pair> path;
  // we're at the goal
  if (isDestination(x1, y1)) {
    cout << "Destination found" << endl;
    return path;
  }

  bool closedList[(int)x_res][(int)y_res];
  memset(closedList, false, sizeof (closedList));

  cell cellDetails[ROW][COL];
  for (int i = 0; i < x_res; i++) {
    for (int j = 0; j < y_res; j++) {
      cellDetails[i][j].f = FLT_MAX;
      cellDetails[i][j].g = FLT_MAX;
      cellDetails[i][j].h = FLT_MAX;
      cellDetails[i][j].parent_i = -1;
      cellDetails[i][j].parent_j = -1;
    }
  }

  // Initialising the parameters of the starting node
  int i, j;
  i = x1, j = y1;
  cellDetails[i][j].f = 0.0;
  cellDetails[i][j].g = 0.0;
  cellDetails[i][j].h = 0.0;
  cellDetails[i][j].parent_i = i;
  cellDetails[i][j].parent_j = j;

  /*
  Create an open list having information as-
  <f, <i, j>>
  where f = g + h,
  and i, j are the row and column index of that cell
  Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
  This open list is implenented as a set of pair of pair.*/
  std::set<pPair> openList;

  // Put the starting cell on the open list and set its
  // 'f' as 0
  openList.insert(std::make_pair (0.0, std::make_pair (i, j)));

  // We set this boolean value as false as initially
  // the destination is not reached.
  bool foundDest = false;

  while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

       /*
        Generating all the 8 successor of this cell

            N.W   N   N.E
              \   |   /
               \  |  /
            W----Cell----E
                 / | \
               /   |  \
            S.W    S   S.E

        Cell-->Popped Cell (i, j)
        N -->  North       (i-1, j)
        S -->  South       (i+1, j)
        E -->  East        (i, j+1)
        W -->  West           (i, j-1)
        N.E--> North-East  (i-1, j+1)
        N.W--> North-West  (i-1, j-1)
        S.E--> South-East  (i+1, j+1)
        S.W--> South-West  (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j))
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                //printf ("The destination cell is found\n");
                path = tracePath (cellDetails);
                foundDest = true;
                return path;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (!closedList[i-1][j] && isUnBlocked(i-1, j))
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = heuristic(i - 1, j);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j].f == FLT_MAX ||
                        cellDetails[i-1][j].f > fNew)
                {
                    openList.insert( std::make_pair(fNew,
                                               std::make_pair(i-1, j)));

                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i + 1, j))
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                //printf("The destination cell is found\n");
                path = tracePath(cellDetails);
                foundDest = true;
                return path;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = heuristic(i+1, j);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i+1][j].f == FLT_MAX ||
                        cellDetails[i+1][j].f > fNew)
                {
                    openList.insert( std::make_pair (fNew, std::make_pair (i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid (i, j+1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j+1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                //printf("The destination cell is found\n");
                path = tracePath(cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j+1] == false &&
                     isUnBlocked (i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = heuristic (i, j+1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j+1].f == FLT_MAX ||
                        cellDetails[i][j+1].f > fNew)
                {
                    openList.insert( std::make_pair(fNew,
                                        std::make_pair (i, j+1)));

                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j-1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j-1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                //printf("The destination cell is found\n");
                path = tracePath(cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j-1] == false &&
                     isUnBlocked(i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = heuristic(i, j-1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i][j-1].f == FLT_MAX ||
                        cellDetails[i][j-1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew,
                                          std::make_pair (i, j-1)));

                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j+1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j+1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j+1].parent_i = i;
                cellDetails[i-1][j+1].parent_j = j;
                //printf ("The destination cell is found\n");
                tracePath (cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j+1] == false &&
                     isUnBlocked(i-1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = heuristic(i-1, j+1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j+1].f == FLT_MAX ||
                        cellDetails[i-1][j+1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew,
                                    std::make_pair(i-1, j+1)));

                    // Update the details of this cell
                    cellDetails[i-1][j+1].f = fNew;
                    cellDetails[i-1][j+1].g = gNew;
                    cellDetails[i-1][j+1].h = hNew;
                    cellDetails[i-1][j+1].parent_i = i;
                    cellDetails[i-1][j+1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i-1, j-1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination (i-1, j-1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j-1].parent_i = i;
                cellDetails[i-1][j-1].parent_j = j;
                //printf ("The destination cell is found\n");
                tracePath (cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j-1] == false &&
                     isUnBlocked(i-1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = heuristic(i-1, j-1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i-1][j-1].f == FLT_MAX ||
                        cellDetails[i-1][j-1].f > fNew)
                {
                    openList.insert( std::make_pair (fNew, std::make_pair (i-1, j-1)));
                    // Update the details of this cell
                    cellDetails[i-1][j-1].f = fNew;
                    cellDetails[i-1][j-1].g = gNew;
                    cellDetails[i-1][j-1].h = hNew;
                    cellDetails[i-1][j-1].parent_i = i;
                    cellDetails[i-1][j-1].parent_j = j;
                }
            }
        }

        //----------- 7th Successor (South-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j+1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j+1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j+1].parent_i = i;
                cellDetails[i+1][j+1].parent_j = j;
                //printf ("The destination cell is found\n");
                tracePath (cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j+1] == false &&
                     isUnBlocked(i+1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = heuristic(i+1, j+1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i+1][j+1].f == FLT_MAX ||
                        cellDetails[i+1][j+1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew,
                                        std::make_pair (i+1, j+1)));

                    // Update the details of this cell
                    cellDetails[i+1][j+1].f = fNew;
                    cellDetails[i+1][j+1].g = gNew;
                    cellDetails[i+1][j+1].h = hNew;
                    cellDetails[i+1][j+1].parent_i = i;
                    cellDetails[i+1][j+1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i+1, j-1) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j-1) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j-1].parent_i = i;
                cellDetails[i+1][j-1].parent_j = j;
                //printf("The destination cell is found\n");
                path = tracePath(cellDetails);
                foundDest = true;
                return path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j-1] == false &&
                     isUnBlocked(i+1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = heuristic(i+1, j-1);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[i+1][j-1].f == FLT_MAX ||
                        cellDetails[i+1][j-1].f > fNew)
                {
                    openList.insert(std::make_pair(fNew,
                                        std::make_pair(i+1, j-1)));

                    // Update the details of this cell
                    cellDetails[i+1][j-1].f = fNew;
                    cellDetails[i+1][j-1].g = gNew;
                    cellDetails[i+1][j-1].h = hNew;
                    cellDetails[i+1][j-1].parent_i = i;
                    cellDetails[i+1][j-1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    return path;
}

// DEBUG




// Assumptions :
// 1) Line is drawn from left to right.
// 2) x1 < x2 and y1 < y2
// 3) Slope of the line is between 0 and 1.
//    We draw a line from lower left to upper
//    right.
// Attribution: Geeks for Geeks
void bresenham(int x1, int y1, int x2, int y2, int tgt_x, int tgt_y)
{
    int m_new = 2 * (x2 - x1);
    int slope_error_new = m_new - (y2 - y1);
    for (int x = x1, y = y1; y <= y2; y++)
    {
        if (x != tgt_x || y != tgt_y) {
            x = clamp(0, x, 207);
            y = clamp(0, y, 207);
            //cout << "MISS @ " << x << ", " << y << endl;
            mtx.lock();
            grid[x][y] -= log_odd_free;
            mtx.unlock();
        }

        // Add slope to increment angle formed
        slope_error_new += m_new;

        // Slope error reached limit, time to
        // increment y and update slope error.
        if (slope_error_new >= 0)
        {
            x++;
            slope_error_new  -= 2 * (y2 - y1);
        }
   }
}

void
callback(Robot* robot)
{
    std::vector<float> angles;

    mtx.lock();
    int x = estimate_x;
    int y = estimate_y;
    mtx.unlock();

    for (auto hit : robot->ranges) {
        if (hit.range <= 2) {
            // get hit info for the occupancy grid
            angles.push_back(hit.angle);

            int angle = hit.angle + (M_PI / 2.0);
            float dx = 0.5 * hit.range * cos(angle);
            float dy = 0.5 * hit.range * sin(angle);
            float abs_x = x + dx;
            float abs_y = y + dy;
            abs_x = abs_x / res;
            abs_y = abs_y / res;
            float adj_x = abs(abs_x - x_offset);
            float adj_y = abs(abs_y - y_offset);
            adj_x = clamp(0, adj_x, 207);
            adj_y = clamp(0, adj_y, 207);

            float start_x = abs(x/res - x_offset);
            float start_y = abs(y/res - y_offset);
            start_x = ceil(start_x);
            start_y = ceil(start_y);
            start_x = clamp(0, start_x, 207);
            start_y = clamp(0, start_y, 207);

            //cout << "HIT: (" << (int)adj_x << ", " << (int)adj_y << ") " << endl;
            mtx.lock();
            grid[(int)adj_x][(int)adj_y] += log_odd_occ;
            mtx.unlock();

            // update misses
            if (start_x > adj_x && start_y > adj_y) {
                bresenham(adj_x, adj_y, start_x, start_y, adj_x, adj_y);
            } else {
                bresenham(start_x, start_y, adj_x, adj_y, adj_x, adj_y);
            }
        }

        if (hit.range < 0.5) {
            cout << "PROXIMITY ALERT!" << endl;
            robot->set_vel(0, 0);
            return;
        }
    }

    int size = angles.size();
    while (size < 7) {
        //cout << "Size: " << size << endl;
        // update all estimated points based on misses
        std::vector<float> angles_raw = {2.35619, 1.57079, 0.785397, 0, -0.785397, -1.57079, -2.35619};
        for (float angle : angles_raw) {
            bool found = false;
            for (float angle_hit : angles) {
               if (abs(angle_hit - angle) < 0.1) {
                    found = true;
               }
            }
            if (!found) {
                //cout << "Updating a miss on " << angle << endl;
                int angle = angle + (M_PI / 2.0);
                float dx = 0.5 * 2 * cos(angle);
                float dy = 0.5 * 2 * sin(angle);
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
        size++;
    }

    // now pathfind - align left, top, right, down
    std::pair<int,int> p = curr_path.top();
    curr_path.pop();
    int tgt_x = round(p.first);
    int tgt_y = round(p.second);

    float east = 1.57;
    float northeast = 0.785;
    float north = 0.0;
    float northwest = -0.785;
    float west = -1.57;
    float southeast = 2.35;
    float south = 3.14;
    float southwest = -2.35;


    cout << "heading to " << tgt_x << ", " << tgt_y << endl;
    if (x < estimate_x && y == estimate_y) {
        cout << "heading up" << endl;
        robot->set_vel(1.0, 1.0); // move up
    } else if (x < estimate_x && y < estimate_y) {
        robot->set_vel()
    }


    return;
}

void
robot_thread(Robot* robot)
{
    robot->do_stuff();
}

void
draw_thread(Robot* robot)
{
    while (true) {
        if (count < 2000) {
            count++;
            observed_x += robot->pos_x;
            observed_y += robot->pos_y;
        } else {
            mtx.lock();
            estimate_x = observed_x / count;
            estimate_y = observed_y / count;
            mtx.unlock();
            count = 0;
            observed_x = 0;
            observed_y = 0;
            clear();
        }

        //cout << "Drawing grid" << endl;
        for (int i = 0; i < 208; i++) {
            for (int j = 0; j < 208; j++) {
                mtx.lock();
                float confidence = grid[i][j];
                mtx.unlock();
                if (confidence > 0.7) {
                    //cout << "Drawn: " << i << ", " << j << " Confidence: " << grid[i][j] << endl;
                    draw_index(j, i);
                }
            }
        }
    }
}

void
loc_thread(Robot* robot)
{
    bool estimating;
    while (true) {
        if (count < 2000) {
            estimating = true;
            count++;
            observed_x += robot->pos_x;
            observed_y += robot->pos_y;
        } else {
            estimating = false;
            mtx.lock();
            estimate_x = observed_x / count;
            estimate_y = observed_y / count;
            mtx.unlock();
            count = 0;
            observed_x = 0;
            observed_y = 0;
      }

      if (estimating) {
          continue;
      } else {
        float abs_x = estimate_x;
        float abs_y = estimate_y;
        abs_x = abs_x / res;
        abs_y = abs_y / res;
        float adj_x = abs(abs_x - x_offset);
        float adj_y = abs(abs_y - y_offset);
        adj_x = clamp(0, adj_x, 207);
        adj_y = clamp(0, adj_y, 207);

        std::stack<Pair> path;
        path = aStarSearch(adj_x, adj_y);

        mtx.lock();
        curr_path = path;
        mtx.unlock();
      }

  }
}


int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);
    std::thread dthr(draw_thread, &robot);
    std::thread lthr(loc_thread, &robot);


    /*
    std::vector<std::tuple<int, int>> points = bresen(0, 0, 3, 3);
    for (int i = 0; i < points.size(); i++) {
        std::tuple<int, int> point = points.at(i);
        cout << std::get<0>(point) << std::get<1>(point)  << endl;
    }
    */


    return viz_run(argc, argv);
}
