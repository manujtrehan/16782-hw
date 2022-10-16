#ifndef PLANNER_FUNCTIONS_H
#define PLANNER_FUNCTIONS_H

#pragma once

#include <iostream>
#include <vector>
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
extern std::uniform_real_distribution<> goalDist;

struct Node
{
    std::vector<double> angles; // vector of joint angles
    double parentDist; // distance to parent node
    int distFromRoot;
    std::shared_ptr<Node> parent; // pointer to parent node

    // declare constructors
    Node();
    Node(const std::vector<double> a);
    Node(const double* a, int size);

    void updateAngles(const double* a, int size);
};

constexpr double epsilon = PI/10; // PI/20; // max angle change in a single step
constexpr double stepSize = PI/100; // PI/180; // collision check interpolation step size
constexpr double goalThresh = PI/8; // PI/90; // goal region threshold. each angle can be off by at max this value in radians
constexpr double goalBias = 0.05; // goal bias probability with which to sample directly towards the goal

extern std::vector<std::shared_ptr<Node> > nodeList;

std::shared_ptr<Node> randomNode(int numJoints);

std::tuple<std::shared_ptr<Node>, double, double> nearestNeighbor(std::shared_ptr<Node> rNode);

std::tuple<double, double> getDistance(
                                std::shared_ptr<Node> n1,
                                std::shared_ptr<Node> n2);

bool linInterp(
        std::shared_ptr<Node> startNode,
        std::shared_ptr<Node> endNode,
        double* map,
        int x_size,
        int y_size,
        double maxAngleDiff);

std::shared_ptr<Node> extendRRT(
                        std::shared_ptr<Node> rNode,
                        double* map,
                        int x_size,
                        int y_size);

bool reachedGoal(
        std::shared_ptr<Node> goal,
        std::shared_ptr<Node> n);

int buildRRT(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints);



#endif