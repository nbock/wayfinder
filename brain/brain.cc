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
float log_odd_occ = 1;
float log_odd_free = 1;
float count = 0.00;
float observed_x = 0.00;
float observed_y = 0.00;
float goal_x = abs( (20.0 / res) - x_offset);
float goal_y = abs( (0.0 / res) - y_offset);
float estimate_x;
float estimate_y;
float tgt_heading = 0;
bool estimating = true;
bool front_clear = true;
bool right_clear = true;
bool left_clear = true;
bool blocked = true;
bool path_found = false;
int tgt_x, tgt_y;
int adj_x, adj_y;

#define ROW 208
#define COL 208


float grid[208][208];

std::mutex mtx;

// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef std::pair<double, std::pair<int, int>> pPair;

std::vector<Pair> curr_path;

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

float distance(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

float heuristic(int x1, int y1) {
    // straight line distance to the goal
    float raw = sqrt( pow((x1 - goal_x), 2) + pow((y1 - goal_y), 2) );

    // we know that each wall adds at least one square more we need to travel
    //float wall = 0;
    float square = 0;
    std::vector<std::tuple<int,int>> points;
    if (x1 > goal_x && y1 > goal_y) {
        points = bresen(x1, y1, goal_x, goal_y);
    } else {
        points = bresen(goal_x, goal_y, x1, y1);
    }

    raw += grid[x1][y1];
    for (int i = 0; i < points.size(); i++) {
        int row = std::get<0>(points[i]);
        int col = std::get<1>(points[i]);
        row = clamp(0, row, 207);
        col = clamp(0, col, 207);

        // SEGFAULT ON THESE?
        square = grid[row][col];
        square += grid[row - 1][col - 1] * 0.5;
        square += grid[row - 1][col + 1] * 0.5;
        square += grid[row - 1][col] * 0.5;
        square += grid[row + 1][col - 1] * 0.5;
        square += grid[row + 1][col + 1] * 0.5;
        square += grid[row + 1][col] * 0.5;
        square += grid[row][col + 1] * 0.5;
        square += grid[row][col - 1] * 0.5;

        raw += square;
    }

    return raw;
}

// A Utility Function to trace the path from the source
// to destination
std::vector<Pair> tracePath(cell cellDetails[][COL])
{
    //printf ("\nThe Path is ");
    int row = goal_x;
    int col = goal_y;

    std::vector<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {
        Path.push_back (std::make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push_back (std::make_pair (row, col));

    return Path;
}

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < 208) &&
           (col >= 0) && (col < 208);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(int row, int col)
{
    // Returns true if the cell is not blocked else false
    mtx.lock();
    float square = grid[row][col];
    mtx.unlock();

    return true;
    if (square <= 0) {
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
        return true;
    else
        return false;
}

// A start function for finding goal
// Adapted from https://www.geeksforgeeks.org/a-search-algorithm/
std::vector<Pair> aStarSearch(int x1, int y1) {

  std::vector<Pair> path_a;
  // we're at the goal
  if (isDestination(x1, y1)) {
    cout << "Destination found" << endl;
    return path_a;
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
                path_a = tracePath (cellDetails);
                foundDest = true;
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
                path_a = tracePath(cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath(cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath(cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath (cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath (cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath (cellDetails);
                foundDest = true;
                return path_a;
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
                path_a = tracePath(cellDetails);
                foundDest = true;
                return path_a;
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
    if (foundDest == false) {
        printf("Failed to find the Destination Cell\n");
        std::vector<Pair> path_empty;
        return path_empty;
    }

    return path_a;
}

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
    if (!path_found) {
        return;
    }

    if (robot->at_goal()) {
        cout << "WIN!" << endl;
        return;
    }
    std::vector<float> angles;

    mtx.lock();
    float x = estimate_x;
    float y = estimate_y;
    mtx.unlock();

    bool stop = false;
    for (auto hit : robot->ranges) {
        if (hit.range <= 100) {
            // get hit info for the occupancy grid
            angles.push_back(hit.angle);
            if ((-0.2 < abs(hit.angle) < 0.2) || ((abs(hit.angle) - 0.785) < 0.2)) {
                if (hit.range <= 1) {
                    mtx.lock();
                    front_clear = false;
                    mtx.unlock();
                }
            } else if ((hit.angle - 1.57) < 0.2) {
                if (hit.range <= 1) {
                    mtx.lock();
                    left_clear = false;
                    mtx.unlock();
                }
            } else if ((abs(hit.angle) - 1.57) < 0.2) {
                if (hit.range <= 1) {
                    mtx.lock();
                    right_clear = false;
                    mtx.unlock();
                }
            }

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
            start_x = round(start_x);
            start_y = round(start_y);
            start_x = clamp(0, start_x, 207);
            start_y = clamp(0, start_y, 207);

            //SEGFAULT HERE?
            mtx.lock();
            adj_x = clamp(1, adj_x, 207);
            adj_y = clamp(1, adj_y, 207);
            grid[(int)adj_x][(int)adj_y] += 100;
            int row = adj_x;
            int col = adj_y;
            grid[row - 1][col - 1] += 100;
            grid[row - 1][col + 1] += 100;
            grid[row - 1][col] += 100;
            grid[row + 1][col - 1] += 100;
            grid[row + 1][col + 1] += 100;
            grid[row + 1][col] += 100;
            grid[row][col + 1] += 100;
            grid[row][col - 1] += 100;
            mtx.unlock();
        }

        if (hit.range < 1.0) {
            stop = true;
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
                if ((-0.2 < abs(angle) < 0.2) || ((abs(angle) - 0.785) < 0.2)) {
                    mtx.lock();
                    front_clear = true;
                    mtx.unlock();
                } else if ((angle - 1.57) < 0.2) {
                    mtx.lock();
                    left_clear = false;
                    mtx.unlock();
                } else if ((abs(angle) - 1.57) < 0.2) {
                    mtx.lock();
                    right_clear = false;
                    mtx.unlock();
                }
                //cout << "Updating a miss on " << angle << endl;
                int angle = angle + (M_PI / 2.0);
                float dx = 0.5 * 2 * cos(angle);
                float dy = 0.5 * 2 * sin(angle);
                float abs_x = x + dx;
                float abs_y = y + dy;
                abs_x = abs_x / res;
                abs_y = abs_y / res;
                int adj_x = round(abs(abs_x - x_offset));
                int adj_y = round(abs(abs_y - y_offset));

                int start_x = round(abs(x/res - x_offset));
                int start_y = round(abs(y/res - y_offset));
            }
        }
        size++;
    }

    bool turning = false;

    if (robot->pos_t < tgt_heading) {
        turning = true;
        robot->set_vel(-2, 2); // turn left
    } else {
        turning = true;
        robot->set_vel(2, -2); // turn right
    }

    if (abs(robot->pos_t - tgt_heading) < 0.1) {
        turning = false;
    }

    if ((-3.1 < robot->pos_t < -2.7) && (tgt_heading == 3.14) ) {
        robot->set_vel(1, -1); // turn right
    } else if ((2.7 < robot->pos_t < 3.1) && (tgt_heading == 3.14)) {
        robot->set_vel(-1, 1); // turn left
    }


    if (tgt_heading == 0 && robot->pos_t < 0 && !left_clear) {
        mtx.lock();
        //grid[tgt_x][tgt_y] += 100;
        mtx.unlock();
        blocked = true;
    } else if (tgt_heading == 0 && robot->pos_t > 0 && !right_clear) {
        mtx.lock();
        //grid[tgt_x][tgt_y] += 100;
        mtx.unlock();
        blocked = true;
    }

    if (!turning && front_clear) {
        robot->set_vel(1.0, 1.0);
    }

    if (!turning && !front_clear){
        robot->set_vel(0, 0);
        mtx.lock();
        //grid[tgt_x][tgt_y] += 10;
        stop = false;
        blocked = true;
        mtx.unlock();
    }

    if (stop) {
        robot->set_vel(0, 0);
        mtx.lock();
        //grid[tgt_x][tgt_y] += 100;
        stop = false;
        mtx.unlock();
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
    std::this_thread::sleep_for (std::chrono::seconds(2));
    int count = 0;
    while (true) {
        std::vector<Pair> drawing;
        mtx.lock();
        drawing = curr_path;
        mtx.unlock();

        for (int i = 0; i < drawing.size(); i++) {
            Pair point = drawing.at(i);
            draw_index(point.second, point.first);
        }
        std::this_thread::sleep_for (std::chrono::seconds(2));
    }
}

void
loc_thread(Robot* robot)
{
    int count = 0;
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
    }
}

void
plan_thread(Robot* robot)
{
    while(true){
        float abs_x = robot->pos_x;
        float abs_y = robot->pos_y;

        abs_x = abs_x / res;
        abs_y = abs_y / res;
        float adj_x = round(abs(abs_x - x_offset));
        float adj_y = round(abs(abs_y - y_offset));
        adj_x = clamp(0, adj_x, 207);
        adj_y = clamp(0, adj_y, 207);

        if (blocked || (adj_x == tgt_x && adj_y == tgt_y)
        || (abs(adj_x - tgt_x) > 2) || (abs(adj_y - tgt_y) > 2)
        || true) {
            clear_screen();
            std::vector<Pair> path;
            while (path.size() == 0) {
                path = aStarSearch(adj_x, adj_y);
            }
            blocked = false;
            mtx.lock();
            curr_path = path;
            path_found = true;
            mtx.unlock();

            mtx.lock();
            std::pair<int,int> p = curr_path.back();
            curr_path.erase(curr_path.end());
            tgt_x = p.first;
            tgt_y = p.second;
            mtx.unlock();


            while (distance(adj_x, adj_y, tgt_x, tgt_y) <= 0) {
                std::pair<int,int> p = curr_path.back();
                curr_path.erase(curr_path.end());
                mtx.lock();
                tgt_x = p.first;
                tgt_y = p.second;
                mtx.unlock();
                if (goal_x == tgt_x && goal_y == tgt_y) {
                    break;
                }
            }
        }

        cout << "Current: " << adj_x << ", " << adj_y << endl;
        cout << "Target: " << tgt_x << ", " << tgt_y << endl;
        cout << "Goal: " << goal_x << ", " << goal_y << endl;
        cout << "Target h: " << heuristic(tgt_x, tgt_y) << endl;
        cout << "Target wall: " << grid[tgt_x][tgt_y] << endl;
        cout << "Blocked: " << blocked << endl;
        cout << "Current heading: " << robot->pos_t << endl;
        cout << "Desired heading " << tgt_heading << endl;

        float west = 1.57;
        float northwest = 0.785;
        float north = 0.0;
        float northeast = -0.785;
        float east = -1.57;
        float southwest = 2.35;
        float south = 3.14;
        float southeast = -2.35;

        // find what direction we want to go
        mtx.lock();
        if (tgt_x < adj_x && tgt_y == adj_y) {
            tgt_heading = north;
        } else if (tgt_x < adj_x && tgt_y < adj_y) {
            tgt_heading = northwest; // northwest
        } else if (tgt_x == adj_x && tgt_y < adj_y) {
            tgt_heading = west;
        } else if (tgt_x > adj_x && tgt_y < adj_y) {
            tgt_heading = southwest; // southwest
        } else if (tgt_x > adj_x && tgt_y == adj_y) {
            tgt_heading = south;
        } else if (tgt_x > adj_x && tgt_y > adj_y) {
            tgt_heading = southeast; // southeast
        } else if (tgt_x == adj_x && tgt_y > adj_y) {
            tgt_heading = east;
        } else if (tgt_x < adj_x && tgt_y > adj_y) {
            tgt_heading = northeast; // northeast
        }
        mtx.unlock();
    }
}



int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    std::thread lthr(loc_thread, &robot);
    std::thread rthr(robot_thread, &robot);
    std::thread pthr(plan_thread, &robot);
    std::thread dthr(draw_thread, &robot);

    return viz_run(argc, argv);
}
