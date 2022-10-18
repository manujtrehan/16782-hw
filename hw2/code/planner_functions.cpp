#include "planner_functions.h"

/******* Original Functions *******/

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}


/******* End *******/



// globals - definition
// std::default_random_engine re; //random number generation
std::random_device re; //random number generation
std::mt19937 gen(re());
std::mt19937 goalGen(re());
std::uniform_real_distribution<> sampleJoint(0, 2*PI);
std::uniform_real_distribution<> goalDist(0, 1);
std::vector<std::shared_ptr<Node> > nodeList;
std::vector<std::vector<std::shared_ptr<Node> > > nodeList_AB(2); // RRT Connect - list of 2
double** plan = NULL;

Node::Node() : parentDist(__DBL_MAX__), parent(nullptr), distFromRoot(INT_MAX), cost(0) {}

Node::Node(const std::vector<double> a) : angles(a), parentDist(__DBL_MAX__), parent(nullptr), distFromRoot(INT_MAX), cost(0) {}

Node::Node(const double* a, int size) : parentDist(__DBL_MAX__), parent(nullptr), distFromRoot(INT_MAX), cost(0)
{
    angles = std::vector<double>(a, a + size); // convert a double array to a vector
}

void Node::updateAngles(const double* a, int size)
{
	angles = std::vector<double>(a, a + size); // convert a double array to a vector
}

std::shared_ptr<Node> randomNode(int numJoints)
{
    std::shared_ptr<Node> n = std::make_shared<Node>();
    
    for(int i = 0; i < numJoints; ++i) // sample angles for all joints
    {
        n->angles.push_back(sampleJoint(gen));
    }
    
    return n;
}

std::tuple<double, double> getDistance(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)
{
    double sum = 0, maxAngleDiff = 0, angleDiff;

    for(int i = 0; i < n1->angles.size(); ++i)
    {
        angleDiff = std::fabs(n1->angles[i] - n2->angles[i]);
        if(angleDiff > maxAngleDiff) maxAngleDiff = angleDiff;
        sum += angleDiff;
    }

    return std::make_tuple(sum, maxAngleDiff);
}

std::tuple<std::shared_ptr<Node>, double, double> nearestNeighbor(std::shared_ptr<Node> rNode, std::vector<std::shared_ptr<Node> >& nodes)
{
    std::shared_ptr<Node> nearest;
    double minDist = __DBL_MAX__, newDist, angleDiff, maxAngleDiff;

    for(std::shared_ptr<Node> i : nodes)
    {
        std::tie(newDist, angleDiff) = getDistance(i, rNode);
        if(minDist > newDist)
        {
            minDist = newDist;
            maxAngleDiff = angleDiff;
            nearest = i;
        }
    }

    return std::make_tuple(nearest, minDist, maxAngleDiff);
}

std::vector<std::tuple<int, double, double> > cheapestNeighbors(std::shared_ptr<Node> newNode)
{
	double newDist, maxAngleDiff;
	int numJoints = newNode->angles.size();
	std::vector<std::tuple<int, double, double> > indices;
	for(int i = 0; i < nodeList.size(); ++i)
	{
		std::tie(newDist, maxAngleDiff) = getDistance(nodeList[i], newNode);
		if(newDist < (numJoints * searchRadius)) // store all nodes within the search radius - store (index, sum, maxAngleDiff)
		{
			indices.push_back(std::make_tuple(i, newDist, maxAngleDiff));
		}
	}
	return indices;
}

std::tuple<bool, bool> linInterp(
		std::shared_ptr<Node> startNode,
		std::shared_ptr<Node> newNode,
		double* map,
		int x_size,
		int y_size,
		double maxAngleDiff)
{
    int numSamples = (int) std::ceil(maxAngleDiff/stepSize);
	int numJoints = startNode->angles.size();
    double ratio;
	double *angles = new double[numJoints];
	double *oldAngles = new double[numJoints];

	// if(numSamples == 0) return false; // stuck at start node. resample

	// std::cout << "4 " << numSamples << " " << maxAngleDiff << std::endl;
	bool reached = true;
    for (int i = 1; i <= numSamples; ++i){
		ratio = (double) ((double) i / (double) numSamples);
		for(int j = 0; j < numJoints; ++j)
		{
            angles[j] = startNode->angles[j] + ratio*(newNode->angles[j] - startNode->angles[j]);
        }
		// std::cout << "5" << std::endl;
		if(IsValidArmConfiguration(angles, numJoints, map, x_size, y_size))
		{
			// std::cout << "6" << std::endl;
			std::copy(&angles[0], (&angles[0] + numJoints), &oldAngles[0]); // store previous angles: oldAngles = angles
		}
		else if(i == 1) // did not update angles at all. stuck at start node. resample
		{
			return std::make_tuple(false, false);
		}
		else
		{
			reached = false;
			break;
		}
    }

	// node to add has angles = oldAngles. update angles of newNode
	newNode->updateAngles(oldAngles, numJoints);

	delete[] angles;
	delete[] oldAngles;

	return std::make_tuple(true, reached);
}

std::tuple<std::shared_ptr<Node>, bool> extendRRT(
						std::shared_ptr<Node> rNode,
						std::vector<std::shared_ptr<Node> >& nodes,
						double* map,
						int x_size,
						int y_size,
						double eps)
{
    // get nearest neighbor
	// std::cout << "2" << std::endl;
    std::shared_ptr<Node> nearestNode, newNode;
    double distance, maxAngleDiff;
    std::tie(nearestNode, distance, maxAngleDiff) = nearestNeighbor(rNode, nodes);

    if(maxAngleDiff > eps) // new node too far, take a step of amount epsilon
    {
        double step = eps/maxAngleDiff, angle; // ratio of step to take for all joints
        std::vector<double> newAngles;
        for(int i = 0; i < nearestNode->angles.size(); ++i)
        {
            angle = nearestNode->angles[i] + step*(rNode->angles[i] - nearestNode->angles[i]); // take a proportional step for each joint
            newAngles.push_back(angle);
        }
        newNode = std::make_shared<Node>(newAngles); // create a new node with the new angle set
        maxAngleDiff = eps; // update max angle difference
    }
    else // rNode is close enough, consider it as the new node to add
    {
        newNode = rNode;
    }
	// std::cout << "3" << std::endl;
    // interpolate and check collisions until the new node
	bool extended, reached;
	std::tie(extended, reached) = linInterp(nearestNode, newNode, map, x_size, y_size, maxAngleDiff);
    if(!extended)
	{
		return std::make_tuple(nullptr, false); // if could not interpolate at all due to obstacles, return nullptr. continue main loop iteration
	}

	// set parent and distance
	std::tie(distance, maxAngleDiff) = getDistance(nearestNode, newNode);
	newNode->parentDist = distance;
	newNode->parent = nearestNode;
	newNode->cost = nearestNode->cost + distance;
	newNode->distFromRoot = nearestNode->distFromRoot + 1;

	nodes.push_back(newNode); // add new node to node list

    return std::make_tuple(newNode, reached);
}

bool reachedGoal(std::shared_ptr<Node> goal, std::shared_ptr<Node> n)
{
	double sum, maxAngleDiff;
	std::tie(sum, maxAngleDiff) = getDistance(goal, n);

	return (maxAngleDiff <= goalThresh);
}

void rewireRRTStar(
            std::shared_ptr<Node> newNode,
			double* map,
			int x_size,
			int y_size)
{
	// get a list of all nearest neighbors to the new node in a radius. returns vector of (index, sum, maxAngleDiff)
	std::vector<std::tuple<int, double, double> > indices = cheapestNeighbors(newNode);
	// std::cout << nodeList.size() << " " << indices.size() << std::endl;

	bool extended, reached, update = false;
	int index, minIndex;
	double c, sum, maxAngleDiff, minCost = newNode->cost;
	std::tuple<int, double, double> min;
	std::shared_ptr<Node> temp = std::make_shared<Node>(newNode->angles); // temp node so that newNode doesn't get updated in linInterp

	for(auto i : indices)
	{
		std::tie(index, sum, maxAngleDiff) = i;
		temp->angles = newNode->angles;

		// try connecting the new node with all nodes in the nearest list
		std::tie(extended, reached) = linInterp(nodeList[index], temp, map, x_size, y_size, maxAngleDiff);
		if(reached)
		{
			c = nodeList[index]->cost + sum;
			if(c < minCost) // found a cheaper node
			{
				std::cout << "yes" << std::endl;
				min = i;
				minCost = c;
				update = true;
			}
		}
	}

	std::tie(minIndex, sum, maxAngleDiff) = min;

	if(update) // update newNode params and rewire tree only if a cheaper neighbor was found
	{
		newNode->parent = nodeList[minIndex];
		newNode->parentDist = sum;
		newNode->cost = minCost;
		newNode->distFromRoot = nodeList[minIndex]->distFromRoot + 1;

		// rewire tree
		for(auto i : indices)
		{
			std::tie(index, sum, maxAngleDiff) = i;
			if(index == minIndex) continue;

			temp->angles = nodeList[index]->angles;
			// try connecting the new node with all nodes in the nearest list. newNode is now the start node - however, it won't make a diff to linInterp
			std::tie(extended, reached) = linInterp(newNode, temp, map, x_size, y_size, maxAngleDiff);
			c = newNode->cost + sum;
			if(reached and (c < nodeList[index]->cost))
			{
				// rewire node
				nodeList[index]->parent = newNode;
				nodeList[index]->parentDist = sum;
				nodeList[index]->distFromRoot = newNode->distFromRoot + 1;
				nodeList[index]->cost = c;
			}
		}
	}

}

int buildRRT(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints)
{
    std::shared_ptr<Node> start = std::make_shared<Node>(armstart_anglesV_rad, numJoints);
    std::shared_ptr<Node> goal = std::make_shared<Node>(armgoal_anglesV_rad, numJoints);
	start->distFromRoot = 0;
    nodeList.push_back(start);

    bool reached;
	std::shared_ptr<Node> rNode, extendedNode;
    for(int iter = 0; iter < maxIter; ++iter)
    {
        // sample a new random node. set it equal to the goal with some goal bias probability
		if(goalDist(goalGen) <= goalBias)
		{
			rNode = goal;
			// std::cout << iter << " " << rNode->angles[0] << std::endl;
		}
        else rNode = randomNode(numJoints);
		// std::cout << rNode->angles[0] << " " << rNode->angles[1] << " " << rNode->angles[2] << " " << rNode->angles[3] << " " << rNode->angles[4] << std::endl;
        
		// try extending
		std::tie(extendedNode, reached) = extendRRT(rNode, nodeList, map, x_size, y_size, epsilon);
		// std::cout << "7" << std::endl;
		if(extendedNode == nullptr) continue; // if extend returns nullptr: continue iteration and resample

		// check if goal reached
		if(reachedGoal(goal, extendedNode))
		{
			// connect to goal
			goal->parent = extendedNode;
			goal->distFromRoot = extendedNode->distFromRoot + 1;
			nodeList.push_back(goal);

			// backtrack
			std::shared_ptr<Node> s = goal;
			int length = s->distFromRoot + 1;
			plan = new double*[length];
			for(int i = (length - 1); i >= 0; --i)
			{
				plan[i] = new double[numJoints];
				std::copy(s->angles.begin(), s->angles.end(), plan[i]);
				s = s->parent;
			}

			return length;
		}
    }

	return 0;
}


int buildRRTConnect(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints)
{
	std::shared_ptr<Node> start = std::make_shared<Node>(armstart_anglesV_rad, numJoints);
    std::shared_ptr<Node> goal = std::make_shared<Node>(armgoal_anglesV_rad, numJoints);
	start->distFromRoot = 0;
	goal->distFromRoot = 0;
    nodeList_AB[0].push_back(start);
    nodeList_AB[1].push_back(goal);

	std::shared_ptr<Node> rNode;
	bool reached;
	int a = 1, b = 0;
	std::vector<std::shared_ptr<Node> > extendedList(2);

	for(int iter = 0; iter < maxIter; ++iter)
	{
		// flip tree indices
		a = 1 - a;
		b = 1 - b;

		// sample a new random node
		rNode = randomNode(numJoints);

		// try extending
		std::tie(extendedList[a], reached) = extendRRT(rNode, nodeList_AB[a], map, x_size, y_size, epsilonConnect);
		if(extendedList[a] == nullptr) continue;
		else
		{
			// try and connect to the other tree
			extendedList[b] = std::make_shared<Node>(extendedList[a]->angles);
			std::tie(extendedList[b], reached) = extendRRT(extendedList[b], nodeList_AB[b], map, x_size, y_size, epsilonConnect);
			if(reached)
			{
				// backtrack
				std::shared_ptr<Node> s = extendedList[0];
				int l1 = extendedList[0]->distFromRoot + 1, l2 = extendedList[1]->parent->distFromRoot + 1;
				plan = new double*[l1 + l2];
				for(int i = (l1 - 1); i >= 0; --i) // backtrack tree A
				{
					plan[i] = new double[numJoints];
					std::copy(s->angles.begin(), s->angles.end(), plan[i]);
					s = s->parent;
				}
				s = extendedList[1]->parent;
				for(int i = l1; i < (l1 + l2); ++i) // backtrack tree B
				{
					plan[i] = new double[numJoints];
					std::copy(s->angles.begin(), s->angles.end(), plan[i]);
					s = s->parent;
				}

				return (l1 + l2);
			}
		}
	}

	return 0;
}

int buildRRTStar(
		double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int maxIter,
        int numJoints)
{
	std::shared_ptr<Node> start = std::make_shared<Node>(armstart_anglesV_rad, numJoints);
    std::shared_ptr<Node> goal = std::make_shared<Node>(armgoal_anglesV_rad, numJoints);
	start->distFromRoot = 0;
    nodeList.push_back(start);

	bool reached;
	std::shared_ptr<Node> rNode, extendedNode;
    for(int iter = 0; iter < maxIter; ++iter)
	{
		// sample a new random node. set it equal to the goal with some goal bias probability
		if(goalDist(goalGen) <= goalBias)
		{
			rNode = goal;
		}
        else rNode = randomNode(numJoints);

		// try extending, get the new node which is added
		std::tie(extendedNode, reached) = extendRRT(rNode, nodeList, map, x_size, y_size, epsilon);
		if(extendedNode == nullptr) continue; // if extend returns nullptr: continue iteration and resample

		// rewire tree
		rewireRRTStar(extendedNode, map, x_size, y_size);

		// check if goal reached
		if(reachedGoal(goal, extendedNode))
		{
			// connect to goal
			goal->parent = extendedNode;
			goal->distFromRoot = extendedNode->distFromRoot + 1;
			nodeList.push_back(goal);

			// backtrack
			std::shared_ptr<Node> s = goal;
			int length = s->distFromRoot + 1;
			plan = new double*[length];
			for(int i = (length - 1); i >= 0; --i)
			{
				plan[i] = new double[numJoints];
				std::copy(s->angles.begin(), s->angles.end(), plan[i]);
				s = s->parent;
			}

			return length;
		}
	}

	return 0;
}