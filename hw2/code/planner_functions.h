#ifndef PLANNER_FUNCTIONS_H
#define PLANNER_FUNCTIONS_H

#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <unordered_set>
#include <set>
#include <queue>
#include <memory>
#include <random>
#include <tuple>
#include <limits.h>


/******* Original Functions *******/

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#define PI 3.141592654

extern double** plan;

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);
void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
void get_current_point(bresenham_param_t *params, int *x, int *y);
int get_next_point(bresenham_param_t *params);
int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map, int x_size, int y_size);
int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map, int x_size, int y_size);

/******* End *******/


// extern std::default_random_engine re; //random number generation
extern std::random_device re; //random number generation
extern std::mt19937 gen;
extern std::mt19937 goalGen;
extern std::uniform_real_distribution<> sampleJoint;
extern std::uniform_real_distribution<> goalDist; // goal bias sampler

struct Node
{
    std::vector<double> angles; // vector of joint angles
    int distFromRoot; // number of nodes between root and current node - for allocating double** plan
    double cost; // cost from root for RRT* - also used as g-val for A* in PRM
    std::shared_ptr<Node> parent; // pointer to parent node

    // PRM variables
    int index;
    double h;
    double f;
    std::vector<std::pair<int, double> > neighbors; // vector of children. stores a pair of (index, distance/cost)

    // declare constructors
    Node();
    Node(const std::vector<double> a);
    Node(const double* a, int size);

    void updateAngles(const double* a, int size);
};

// constants
constexpr double epsilon = PI/10; // PI/20; // max angle change in a single step
constexpr double epsilonConnect = __DBL_MAX__; // RRT Connect has infinite epsilon
constexpr double stepSize = PI/90; // PI/180; // collision check interpolation step size
constexpr double goalThresh = PI/6; // PI/90; // goal region threshold. each angle can be off by at max this value in radians
constexpr double goalBias = 0.05; // goal bias probability with which to sample directly towards the goal
constexpr double searchRadius = PI/60; // RRT* radius (per joint) to search around nearest node - total radius = numJoints * radius
constexpr double searchRadiusPRM = PI/10; // RRT* radius (per joint) to search around nearest node - total radius = numJoints * radius
constexpr int nbrLimit = 10; // negihbor limit for connecting in the prm graph

// global variables
extern std::vector<std::shared_ptr<Node> > nodeList;
extern std::vector<std::vector<std::shared_ptr<Node> > > nodeList_AB; // RRT Connect - vector of 2 node lists

static auto compare = [](const std::shared_ptr<Node>& n1, const std::shared_ptr<Node>& n2)
{
    return n1->f > n2->f;
};
extern std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node> >, decltype(compare)> openQueue; // priority queue for A*
extern std::unordered_set<int> closed; // closed list for A*

std::shared_ptr<Node> randomNode(int numJoints);

std::tuple<double, double> getDistance(
                                std::shared_ptr<Node> n1,
                                std::shared_ptr<Node> n2);

std::tuple<std::shared_ptr<Node>, double, double> nearestNeighbor(
                                                        std::shared_ptr<Node> rNode,
                                                        std::vector<std::shared_ptr<Node> >& nodes);

std::list<std::tuple<int, double, double> > cheapestNeighbors(std::shared_ptr<Node> newNode, double searchRad);

// comparison function to compare/order based on cost/sum
static auto cmpCost = [](const std::tuple<int, double, double>& a, const std::tuple<int, double, double>& b) { return std::get<1>(a) < std::get<1>(b); };
bool connectToNearestFree(
            std::shared_ptr<Node> rNode,
			std::vector<std::shared_ptr<Node> >& nodes,
			double* map,
			int x_size,
			int y_size,
			int numJoints,
			bool setHeuristic);

std::tuple<bool, bool> linInterp(
        std::shared_ptr<Node> startNode,
        std::shared_ptr<Node> endNode,
        double* map,
        int x_size,
        int y_size,
        double maxAngleDiff);

std::tuple<std::shared_ptr<Node>, bool> extendRRT(
                        std::shared_ptr<Node> rNode,
                        std::vector<std::shared_ptr<Node> >& nodes,
                        double* map,
                        int x_size,
                        int y_size,
                        double eps);

bool reachedGoal(
        std::shared_ptr<Node> goal,
		std::shared_ptr<Node> n,
		double* map,
		int x_size,
		int y_size);

void rewireRRTStar(
            std::shared_ptr<Node> newNode,
            double* map,
			int x_size,
			int y_size);

int computePath(std::shared_ptr<Node> goal);

int buildRRT(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints);

int buildRRTConnect(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints);

int buildRRTStar(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints);

int buildPRM(
        double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int numNodes,
        int numJoints);



#endif
