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
std::uniform_real_distribution<> goalDist(0, 1); // goal bias distribution
std::vector<std::shared_ptr<Node> > nodeList; // main nodelist
std::vector<std::vector<std::shared_ptr<Node> > > nodeList_AB(2); // RRT Connect - list of 2 nodelists for swapping b/w A and B
std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node> >, decltype(compare)> openQueue(compare); // priority queue for A*
std::unordered_set<int> closed; // closed list for A*
double** plan = NULL; // double ** to store joint angles at each step of the plan

// struct constructors and definitions
Node::Node() : parent(nullptr), distFromRoot(INT_MAX), cost(__DBL_MAX__) {}

Node::Node(const std::vector<double> a) : angles(a), parent(nullptr), distFromRoot(INT_MAX), cost(__DBL_MAX__) {}

Node::Node(const double* a, int size) : parent(nullptr), distFromRoot(INT_MAX), cost(__DBL_MAX__)
{
    angles = std::vector<double>(a, a + size); // convert a double array to a vector
}

void Node::updateAngles(const double* a, int size) // update angles of an existing struct
{
	angles = std::vector<double>(a, a + size); // convert a double array to a vector
}

// function definitions

std::shared_ptr<Node> randomNode(int numJoints) // sample and create a new random node struct object
{
    std::shared_ptr<Node> n = std::make_shared<Node>();
    
    for(int i = 0; i < numJoints; ++i) // sample angles for all joints
    {
        n->angles.push_back(sampleJoint(gen));
    }
    
    return n;
}

std::tuple<double, double> getDistance(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) // get distance between 2 nodes. returns sum of absolute diff, and max diff
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

std::tuple<std::shared_ptr<Node>, double, double> nearestNeighbor( // returns the nearest neighbor - returns (node, sum, maxAngleDiff)
													std::shared_ptr<Node> rNode,
													std::vector<std::shared_ptr<Node> >& nodes)
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

std::list<std::tuple<int, double, double> > cheapestNeighbors(std::shared_ptr<Node> newNode, double searchRad) // returns a list of nearest nbrs in a radius
{
	double newDist, maxAngleDiff, minDist = __DBL_MAX__;
	int numJoints = newNode->angles.size();
	std::list<std::tuple<int, double, double> > indices; // (index, sum, maxAngleDiff)
	for(int i = 0; i < nodeList.size(); ++i)
	{
		std::tie(newDist, maxAngleDiff) = getDistance(nodeList[i], newNode);
		if(newDist < (numJoints * searchRad)) // store all nodes within the search radius - store (index, sum, maxAngleDiff)
		{
			if(newDist < minDist) // add to front of list if it is the nearest neighbor till now, so that closest neighbor is in the front
			{
				indices.push_front(std::make_tuple(i, newDist, maxAngleDiff));
				newDist = minDist;
			}
			else indices.push_back(std::make_tuple(i, newDist, maxAngleDiff));
		}
	}
	return indices;
}

bool connectToNearestFree( // for PRM - connects start and goal to the nearest accessible node
			std::shared_ptr<Node> rNode,
			std::vector<std::shared_ptr<Node> >& nodes,
			double* map,
			int x_size,
			int y_size,
			int numJoints,
			bool setHeuristic)
{
	// increasing order set of nearest neighbors - stores a tuple(index, sum/cost, maxAngleDiff)
	std::set<std::tuple<int, double, double>, decltype(cmpCost)> nearest(cmpCost);
    double newDist, maxAngleDiff;

    for(int i = 0; i < nodes.size(); ++i)
    {
        std::tie(newDist, maxAngleDiff) = getDistance(nodes[i], rNode);
		if(newDist < (numJoints * searchRadiusPRM)) // insert in nearest set only if within the search radius
		{
			nearest.insert(std::make_tuple(i, newDist, maxAngleDiff));
		}

		if(setHeuristic) // rNode is goal - set heuristics (goal distance) for all other nodes for A*
		{
			nodes[i]->h = newDist;
		}
    }

	// try connecting rNode (start or goal) to the nearest neighbor in the nearest set
	bool extended, reached = false;
	int index;
	std::shared_ptr<Node> temp = std::make_shared<Node>(rNode->angles); // temp node so that rNode does not get updated in linInterp

	for(auto i : nearest) // iterate over the increasing order set and try connecting one by one - stop when connected
	{
		std::tie(index, newDist, maxAngleDiff) = i;
		std::tie(extended, reached) = linInterp(nodes[index], temp, map, x_size, y_size, maxAngleDiff);

		if(reached)
		{
			// add edge
			rNode->neighbors.push_back({index, newDist});
			nodes[index]->neighbors.push_back({rNode->index, newDist});

			if(setHeuristic) // if rNode is the goal node, set its parent node as well
			{
				rNode->parent = nodes[index];
			}
			else // rNode is the start node, set the parent for its neighbor, as well as distance from parent
			{
				nodes[index]->parent = rNode;
				nodes[index]->distFromRoot = rNode->distFromRoot + 1;
			}

			break;
		}
	}

    return reached;
}

std::tuple<bool, bool> linInterp( // linear interpolation between 2 nodes - try and connect from start to the new node in steps
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

	bool reached = true;
    for (int i = 1; i <= numSamples; ++i){
		ratio = (double) ((double) i / (double) numSamples);
		for(int j = 0; j < numJoints; ++j)
		{
            angles[j] = startNode->angles[j] + ratio*(newNode->angles[j] - startNode->angles[j]);
        }

		if(IsValidArmConfiguration(angles, numJoints, map, x_size, y_size))
		{
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
	if(!reached) newNode->updateAngles(oldAngles, numJoints);

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
        newNode = std::make_shared<Node>(rNode->angles);
    }
    // interpolate and check collisions until the new node
	bool extended, reached;
	std::tie(extended, reached) = linInterp(nearestNode, newNode, map, x_size, y_size, maxAngleDiff);
    if(!extended)
	{
		return std::make_tuple(nullptr, false); // if could not interpolate at all due to obstacles, return nullptr. continue main loop iteration
	}

	// set parent and distance
	std::tie(distance, maxAngleDiff) = getDistance(nearestNode, newNode);
	newNode->parent = nearestNode;
	newNode->cost = nearestNode->cost + distance;
	newNode->distFromRoot = nearestNode->distFromRoot + 1;

	nodes.push_back(newNode); // add new node to node list

    return std::make_tuple(newNode, reached);
}

bool reachedGoal(
		std::shared_ptr<Node> goal,
		std::shared_ptr<Node> n,
		double* map,
		int x_size,
		int y_size)
{
	double sum, maxAngleDiff;
	std::tie(sum, maxAngleDiff) = getDistance(goal, n);

	bool extended, reached = false;
	if(sum <= goal->angles.size()*goalThresh) // interpolate and check only if the node is within threshold distance of the goal
	{
		std::shared_ptr<Node> temp = std::make_shared<Node>(n->angles); // use temp node so that linInterp does not change node n angles

		std::tie(extended, reached) = linInterp(goal, temp, map, x_size, y_size, maxAngleDiff);
	}

	return reached;
}

void rewireRRTStar(
            std::shared_ptr<Node> newNode,
			double* map,
			int x_size,
			int y_size)
{
	// get a list of all nearest neighbors to the new node in a radius. returns list of (index, sum, maxAngleDiff). nearest neighbor is the 1st element
	std::list<std::tuple<int, double, double> > indices = cheapestNeighbors(newNode, searchRadius);
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
				min = i;
				minCost = c;
				update = true;
			}
		}
	}

	if(update) // update newNode params only if a cheaper neighbor was found
	{
		std::tie(minIndex, sum, maxAngleDiff) = min;
		newNode->parent = nodeList[minIndex];
		newNode->cost = minCost;
		newNode->distFromRoot = nodeList[minIndex]->distFromRoot + 1;
	}
	else // set minIndex to nearest neighbor which is present at the front of the list
	{
		std::tie(minIndex, sum, maxAngleDiff) = indices.front();
	}

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
			nodeList[index]->distFromRoot = newNode->distFromRoot + 1;
			nodeList[index]->cost = c;
		}
	}
}

int computePath(std::shared_ptr<Node> goal)
{
	int length = 0;
	while(!openQueue.empty())
	{
		std::shared_ptr<Node> s = openQueue.top();
		openQueue.pop();

		if(closed.find(s->index) != closed.end()) continue; // skip if already visited
		closed.insert(s->index); // add to closed list

		if(s->index == goal->index)
		{
			// goal found. backtrack
			length = s->distFromRoot + 1;
			plan = new double*[length];
			for(int i = (length - 1); i >= 0; --i)
			{
				plan[i] = new double[goal->angles.size()];
				std::copy(s->angles.begin(), s->angles.end(), plan[i]);
				s = s->parent;
			}

			return length;
		}

		int index;
		double sum;
		// std::cout << s->neighbors.size() << std::endl;
		for(auto i : s->neighbors)
		{
			std::tie(index, sum) = i;
			if(closed.find(index) == closed.end())
			{
				if(nodeList[index]->cost > (s->cost + sum))
				{
					nodeList[index]->cost = s->cost + sum;
					nodeList[index]->f = nodeList[index]->cost + nodeList[index]->h;
					nodeList[index]->parent = s;
					nodeList[index]->distFromRoot = s->distFromRoot + 1;
					openQueue.push(nodeList[index]);
				}
			}
		}
	}

	return length;
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
		}
        else rNode = randomNode(numJoints);
        
		// try extending
		std::tie(extendedNode, reached) = extendRRT(rNode, nodeList, map, x_size, y_size, epsilon);

		if(extendedNode == nullptr) continue; // if extend returns nullptr: continue iteration and resample

		// check if goal reached
		if(reachedGoal(goal, extendedNode, map, x_size, y_size))
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
	start->cost = 0;
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
		if(reachedGoal(goal, extendedNode, map, x_size, y_size))
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

int buildPRM(
        double* map,
        double* armstart_anglesV_rad,
		double* armgoal_anglesV_rad,
		int x_size,
		int y_size,
        int numNodes,
        int numJoints)
{
	std::shared_ptr<Node> rNode, temp = std::make_shared<Node>();
	double* angArr = new double[numJoints];
	std::list<std::tuple<int, double, double> > indices;
	int index;
	double sum, maxAngleDiff;
	bool extended, reached;

	while(nodeList.size() < numNodes) // build PRM graph until max number of nodes reached
	{
		// sample a random node
		rNode = randomNode(numJoints);

		std::copy(rNode->angles.begin(), rNode->angles.end(), angArr); // copy to a double* for checking validity
		if(!IsValidArmConfiguration(angArr, numJoints, map, x_size, y_size)) continue; // if sample is invalid, resample

		rNode->index = nodeList.size(); // set index of node and push it into the node list
		nodeList.push_back(rNode);

		indices = cheapestNeighbors(rNode, searchRadiusPRM); // get a list of neighbors within a radius

		for(auto i : indices)
		{
			if(rNode->neighbors.size() == nbrLimit) break; // no more connections if neighbor limit reached
			std::tie(index, sum, maxAngleDiff) = i;
			if(nodeList[index]->neighbors.size() < nbrLimit) // connect to near-node only if that node has not crossed the neighbor limit
			{
				temp->angles = rNode->angles; // temp node so that rNode does not get changed in linInterp

				// try connecting the new node (temp) with the corresponding 'index' node in the nearest list
				std::tie(extended, reached) = linInterp(nodeList[index], temp, map, x_size, y_size, maxAngleDiff);

				if(reached)
				{
					// add edge - add to neighbor lists
					nodeList[index]->neighbors.push_back({rNode->index, sum});
					rNode->neighbors.push_back({index, sum});
				}
			}
		}
	}

	// search for start and goal using A*

	// connect start and goal to nearest nodes
	std::shared_ptr<Node> start = std::make_shared<Node>(armstart_anglesV_rad, numJoints);
	std::shared_ptr<Node> goal = std::make_shared<Node>(armgoal_anglesV_rad, numJoints);

	// set index, cost and parent dist for start node - don't push in nodelist yet otherwise connectToNearestFree() will consider it
	start->index = nodeList.size();
	start->cost = 0;
	start->distFromRoot = 0;

    if(!connectToNearestFree(start, nodeList, map, x_size, y_size, numJoints, false)) // try and connect the start to nearest node
	{
		std::cout << "Obstacles near start - resample nodes" << std::endl;
		return 0;
	}

	nodeList.push_back(start); // push start after it has been connected to a nearest neighbor

	// process goal node now. set index and h-val
	goal->index = nodeList.size();
	goal->h = 0;

	if(!connectToNearestFree(goal, nodeList, map, x_size, y_size, numJoints, true)) // try and connect the start to nearest node
	{
		std::cout << "Obstacles near goal - resample nodes" << std::endl;
		return 0;
	}

	nodeList.push_back(goal); // push goal after it has been connected to a nearest neighbor

	// update f for start - do it only after connecting goal since heuristic computation happens while connecting goal
	start->f = start->cost + start->h;
	// update f for neighbor of start - do it only after connecting goal since heuristic computation happens while connecting goal
	index = start->neighbors[0].first;
	nodeList[index]->f = nodeList[index]->cost + nodeList[index]->h;

	// start A* search
	openQueue.push(start); // add start node to the open queue
	int length = computePath(goal);

	return length;
}