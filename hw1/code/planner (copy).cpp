/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <stack>
#include <memory>
#include <vector>
#include <algorithm>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct node
{
    int mapIndex;
    int g;
    int h;
    int f;
    int time;
    std::shared_ptr<node> parent;

    node() : parent(nullptr), g(INT_MAX), f(INT_MAX) {}
    node(int ind, int hVal, int t) : parent(nullptr), g(INT_MAX), f(INT_MAX)
    {
        mapIndex = ind;
        h = hVal;
        time = t;
    }

    // bool operator==(const std::shared_ptr<node>& a) const
    // {
    //     return (mapIndex == a->mapIndex) and (time == a->time);
    // }

    // bool operator<(const std::shared_ptr<node>& a) const
    // {
    //     return f < a->f;
    // }

    // bool operator>(const std::shared_ptr<node>& a) const
    // {
    //     return f > a->f;
    // }
};

// auto compare = [](const std::shared_ptr<node>& n1, const std::shared_ptr<node>& n2)
// {
//     // return n1->f > n2->f;
//     return n1->f < n2->f;
// };

bool nodeCompare(const std::shared_ptr<node> n1, const std::shared_ptr<node> n2)
{
    // return n1->f > n2->f;
    return n1->f < n2->f;
};


// Globals


int prevX, prevY;

static void computeHeuristics(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        double* map,
        int collision_thresh,
        std::unordered_map<int, std::shared_ptr<node> >& nodes,
        std::unordered_map<int, int>& heuristics,
        // std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node> >, decltype(compare)> &openQueue
        // std::set<std::shared_ptr<node>, decltype(compare)> &openQueue
        std::vector<std::shared_ptr<node> >& openQueue
        )
{
    int gX = x_size + 1;
    int gY = y_size + 1;
    int count = 0;
    // mexPrintf("before: %d %d \n", nodes.size(), x_size*y_size);
    while(!openQueue.empty())
    {
        // std::shared_ptr<node> s = openQueue.top();
        // std::shared_ptr<node> s = *openQueue.begin();
        std::shared_ptr<node> s = openQueue[0];
        // closed.insert(s->mapIndex);
        // openQueue.pop();
        openQueue.erase(openQueue.begin());
        // if(heuristics.find(s->mapIndex) == heuristics.end() or heuristics[s->mapIndex] > s->g)
        // {
        heuristics[s->mapIndex] = s->g; // closed list. stores optimal g-vals based on min out of all goals
        // }
        // if(!openQueue.empty())
        // {   
        //     mexPrintf("%d f1: %d f2: %d %d \n", s->mapIndex, s->f, openQueue.front()->f, openQueue.size());
        // }

        int rX = (int)(s->mapIndex % x_size) + 1;
        int rY = (int)(s->mapIndex / x_size) + 1;

        for(int dir = 0; dir < NUMOFDIRS; ++dir)
        {
            int newx = rX + dX[dir];
            int newy = rY + dY[dir];
            int newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);

            if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and heuristics.find(newIndex) == heuristics.end())
            {
                int cost = (int) map[newIndex];
                // mexPrintf("cost: %d index: %d \n", cost, newIndex);
                if((cost >= 0) and (cost < collision_thresh)) // cell is free
                {
                    // mexPrintf("Adding \n");
                    if(nodes.find(newIndex) == nodes.end()) // create a new node, if it does not exist
                    {
                        // ++count;
                        // int h = (int) MAX(abs(newx - gX), abs(newy - gY));
                        std::shared_ptr<node> n = std::make_shared<node>(newIndex, 0, 0);
                        nodes[newIndex] = n;
                        // openQueue.push(n);
                        // openQueue.insert(n);
                        openQueue.push_back(n);
                        // mexPrintf("added: %d \n", openQueue.size());
                    }
                    if(nodes[newIndex]->g > s->g + cost) // compare g values and cost, update parent if needed
                    {
                        // mexPrintf("before: %d, pointer index: %d, found: %d \n", openQueue.size(), nodes[newIndex]->mapIndex, (*(openQueue.find(nodes[newIndex])))->mapIndex);
                        std::shared_ptr<node> t = nodes[newIndex];
                        // openQueue.erase(openQueue.find(nodes[newIndex]));
                        openQueue.erase(std::find_if(openQueue.begin(), openQueue.end(), [t] (const std::shared_ptr<node>& d) { return d->mapIndex == t->mapIndex; }));
                        t->g = s->g + cost;
                        t->f = t->g + t->h;
                        t->parent = s;
                        openQueue.insert(std::upper_bound(openQueue.begin(), openQueue.end(), t, nodeCompare), t);
                        // if(!openQueue.insert(nodes[newIndex]).second) ++count;
                        // openQueue.push(nodes[newIndex]);
                        // std::make_heap(const_cast<std::shared_ptr<node>* >(&openQueue.top()),
                        //                 const_cast<std::shared_ptr<node>* >(&openQueue.top()) + openQueue.size(),
                        //                 compare);
                        // mexPrintf("after: %d, found: %d \n", openQueue.size(), (*(openQueue.find(nodes[newIndex])))->mapIndex);
                    }
                }
            }
        }
    }
    // mexPrintf("after: %d, count: %d \n", nodes.size(), count);
}

static void computePath(
        int x_size,
        int y_size,
        int dX[],
        int dY[],
        double* map,
        int collision_thresh,
        double* target_traj,
        int target_steps,
        std::unordered_map<int, int>& goals,
        std::unordered_map<int, int>& heuristics,
        std::unordered_map<int, std::shared_ptr<node> >& nodes,
        std::unordered_set<int>& closed,
        // std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node> >, decltype(compare)> &openQueue,
        // std::set<std::shared_ptr<node>, decltype(compare)> &openQueue,
        std::vector<std::shared_ptr<node> >& openQueue,
        std::stack<int>& actionStack
        )
{
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // int goalIndex = GETMAPINDEX(goalposeX, goalposeY, x_size, y_size);
    int goalIndex = -1;
    // int count = 0;
    while(!openQueue.empty())
    {
        // ++count;
        int newx, newy, newIndex, digits, newIndexForMap, cost;
        // std::shared_ptr<node> s = openQueue.top();
        std::shared_ptr<node> s = openQueue[0];
        // std::shared_ptr<node> s = *(openQueue.begin());
        // openQueue.pop();
        openQueue.erase(openQueue.begin());
        digits = (s->time == 0) ? 0 : (int)(std::log10(s->time) + 1);
        newIndexForMap = s->mapIndex * ((int)std::pow(10, digits)) + s->time;
        closed.insert(newIndexForMap);

        int rX = (int)(s->mapIndex % x_size) + 1;
        int rY = (int)(s->mapIndex / x_size) + 1;

        // actionStack.push(s->mapIndex);
        // return;
        mexPrintf("%d %d %d %d %d \n", rX, rY, s->time, s->f, nodes.size());
        // if(goals.find(s->mapIndex) != goals.end() and s->time <= goals[s->mapIndex])
        if(s->mapIndex == goalIndex) // goal reached, add all to action stack and return. remove the common goal
        {
            s = s->parent;
            while(s)
            {
                actionStack.push(s->mapIndex);
                s = s->parent;
            }
            actionStack.pop(); // remove start node
            return;
        }

        else if(goals.find(s->mapIndex) != goals.end() and s->time <= goals[s->mapIndex]) // found one of the goals, connect it to common goal
        {
            int cost = 1; // same cost for all the goals to common goal
            if(nodes.find(goalIndex) == nodes.end()) // create the common goal node if it does not exist
            {
                std::shared_ptr<node> n = std::make_shared<node>(goalIndex, 0, 0);
                nodes[goalIndex] = n;
                openQueue.push_back(n);
                // openQueue.insert(n);
            }
            if(nodes[goalIndex]->g > s->g + cost) // compare g values and cost, update parent if needed
            {
                std::shared_ptr<node> t = nodes[goalIndex];
                // openQueue.erase(openQueue.find(nodes[goalIndex]));
                openQueue.erase(std::find_if(openQueue.begin(), openQueue.end(), [t] (const std::shared_ptr<node>& d) { return (d->mapIndex == t->mapIndex) and (d->time == t->time); }));
                t->g = s->g + cost;
                t->f = t->g + t->h;
                t->parent = s;
                openQueue.insert(std::upper_bound(openQueue.begin(), openQueue.end(), t, nodeCompare), t);
                // openQueue.insert(nodes[goalIndex]);
                // re-order priority queue
                // std::make_heap(const_cast<std::shared_ptr<node>* >(&openQueue.top()),
                //                 const_cast<std::shared_ptr<node>* >(&openQueue.top()) + openQueue.size(),
                //                 compare);
            }
        }
        
        else
        {
            int time = s->time + 1;
            digits = (time == 0) ? 0 : (int)(std::log10(time) + 1);
            if(time > target_steps)
            {
                continue;
            }
            for(int dir = 0; dir < NUMOFDIRS + 1; ++dir)
            {
                newx = rX + dX[dir];
                newy = rY + dY[dir];
                newIndex = (int) GETMAPINDEX(newx, newy, x_size, y_size);
                newIndexForMap = newIndex * ((int)std::pow(10, digits)) + time; // concatenate time value to the end of index for unique key
                
                if(newx >= 1 and newx <= x_size and newy >= 1 and newy <= y_size and closed.find(newIndexForMap) == closed.end())
                {
                    cost = (int) map[newIndex];
                    if((cost >= 0) and (cost < collision_thresh)) // cell is free
                    {
                        if(nodes.find(newIndexForMap) == nodes.end()) // create a new node, if it does not exist
                        {
                            // mexPrintf("new node \n");
                            // int h = (int) floor(sqrt(pow(newx - goalposeX, 2) + pow(newy - goalposeY, 2)));
                            int h = (int) MAX(std::abs(newx - goalposeX), std::abs(newy - goalposeY));
                            // int h = heuristics[newIndex];
                            std::shared_ptr<node> n = std::make_shared<node>(newIndex, h, time);
                            nodes[newIndexForMap] = n;
                            // openQueue.push(n);
                            openQueue.push_back(n);
                            // openQueue.insert(n);
                        }
                        if(nodes[newIndexForMap]->g > s->g + cost) // compare g values and cost, update parent if needed
                        {
                            std::shared_ptr<node> t = nodes[newIndexForMap];
                            // openQueue.erase(openQueue.find(nodes[newIndexForMap]));
                            openQueue.erase(std::find_if(openQueue.begin(), openQueue.end(), [t] (const std::shared_ptr<node>& d) { return (d->mapIndex == t->mapIndex) and (d->time == t->time); }));
                            t->g = s->g + cost;
                            t->f = t->g + 2*t->h;
                            t->parent = s;
                            openQueue.insert(std::upper_bound(openQueue.begin(), openQueue.end(), t, nodeCompare), t);
                            // openQueue.insert(nodes[newIndexForMap]);
                            // re-order priority queue
                            // std::make_heap(const_cast<std::shared_ptr<node>* >(&openQueue.top()),
                            //                 const_cast<std::shared_ptr<node>* >(&openQueue.top()) + openQueue.size(),
                            //                 compare);
                        }
                    }
                }
            }
        }
    }
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // Start

    static bool firstCall = true;
    static std::unordered_map<int, std::shared_ptr<node> > nodes;
    static std::unordered_map<int, int> goals, heuristics;
    static std::unordered_set<int> closed;
    // std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node> >, decltype(compare)> openQueue(compare);
    // std::set<std::shared_ptr<node>, decltype(compare)> openQueue(compare);
    // std::set<std::shared_ptr<node>, std::less<std::shared_ptr<node> > > openQueue;
    static std::vector<std::shared_ptr<node> > openQueue;
    static std::stack<int> actionStack;


    // 8-connected grid
    int dX[NUMOFDIRS + 1] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
    int dY[NUMOFDIRS + 1] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);


    // static bool firstCall = true;
    // static std::unordered_map<int, node*> nodes;
    // static std::unordered_set<int> closed;

    // static std::priority_queue<node*, std::vector<node*>, decltype(compare)> openQueue(compare);
    // static std::stack<int> actionStack;

    prevX = robotposeX;
    prevY = robotposeY;

    if(firstCall) // init s_start, g(start) = 0, add to the open set, and node map
    {
        firstCall = false;

        int gIndex;
        for(int i = 0; i < target_steps; ++i) // setup multi-goal map using backward A*. Calculate heuristics for each cell based on min out of all goals
        {
            // mexPrintf("%d %d \n", i, target_steps);
            gIndex = (int) GETMAPINDEX((int) target_traj[i], (int) target_traj[target_steps + i], x_size, y_size);
            goals[gIndex] = i; // time limit to reach goal
        }

        // int h = (int) MAX(abs(robotposeX-goalposeX), abs(robotposeY-goalposeY));
        std::shared_ptr<node> a = std::make_shared<node>(gIndex, 0, 0);
        a->g = 0;
        a->f = a->g + a->h;
        nodes[gIndex] = a;
        // openQueue.push(a);
        // openQueue.insert(a);
        openQueue.push_back(a);
        computeHeuristics(x_size, y_size, dX, dY, map, collision_thresh, nodes, heuristics, openQueue);
        nodes.clear();
        openQueue.clear();
        closed.clear();
        mexPrintf("%d \n", heuristics.size());



        int h = (int) MAX(abs(robotposeX-goalposeX), abs(robotposeY-goalposeY));
        int index = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
        // int h = heuristics[index];
        // mexPrintf("%d \n", (heuristics.find(index) == heuristics.end()));
        std::shared_ptr<node> b = std::make_shared<node>(index, h, 0);
        b->g = 0;
        b->f = b->g + b->h;
        nodes[index] = b;
        // openQueue.push(b);
        // openQueue.insert(b);
        openQueue.push_back(b);
        // call compute path
        computePath(x_size, y_size, dX, dY, map, collision_thresh, target_traj, target_steps, goals, heuristics, nodes, closed, openQueue, actionStack);
    }
    if(!actionStack.empty())
    {
        int nextIndex = actionStack.top();
        actionStack.pop();
        prevX = (nextIndex % x_size) + 1;
        prevY = (nextIndex / x_size) + 1;
    }

    action_ptr[0] = prevX;
    action_ptr[1] = prevY;

    // End


    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    // double disttotarget;
    // for(int dir = 0; dir < NUMOFDIRS; dir++)
    // {
    //     int newx = robotposeX + dX[dir];
    //     int newy = robotposeY + dY[dir];

    //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    //     {
    //         if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
    //         {
    //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < olddisttotarget)
    //             {
    //                 olddisttotarget = disttotarget;
    //                 bestX = dX[dir];
    //                 bestY = dY[dir];
    //             }
    //         }
    //     }
    // }
    // robotposeX = robotposeX + bestX;
    // robotposeY = robotposeY + bestY;
    // action_ptr[0] = robotposeX;
    // action_ptr[1] = robotposeY;
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}