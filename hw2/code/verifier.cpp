/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

using std::vector;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
  std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
  }
  else {
    printf("Opening file failed! \n");
    throw runtime_error("Opening map file failed!");
  }
  int height, width;
  if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
    throw runtime_error("Invalid loadMap parsing map metadata");
  }
  
  ////// Go through file and add to m_occupancy
  // m_occupancy.resize(height, std::vector<bool>(width, false)); // (H,W)
  double* map = new double[height*width];

  double cx, cy, cz;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      char c;
      do {
        if (fscanf(f, "%c", &c) != 1) {
          throw runtime_error("Invalid parsing individual map data");
        }
      } while (isspace(c));
      if (!(c == '0')) {
        map[y+x*width] = 1;
      } else {
        map[y+x*width] = 0;
      }
    }
  }
  fclose(f);
  return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
    // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
    const std::regex ws_re(delim);
    return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
  vector<string> vals = split(str, ","); // Don't need to worry about trailing comma like in python
  double* ans = new double[vals.size()];
  for (int i = 0; i < vals.size(); ++i) {
    ans[i] = std::stod(vals[i]);
  }
  return ans;
}

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


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
  //take the nearest cell
  *pX = (int)(x/(double)(cellsize));
  if( x < 0) *pX = 0;
  if( *pX >= x_size) *pX = x_size-1;

  *pY = (int)(y/(double)(cellsize));
  if( y < 0) *pY = 0;
  if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
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

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
       int x_size, int y_size)

{
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
       int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
   //iterate through all the links starting with the base
  x1 = ((double)x_size)/2.0;
    y1 = 0;
  for(i = 0; i < numofDOFs; i++)
  {
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

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

/** Your final solution will be verified by this script which will
 * send the default 5 arguments:
 *    map, numOfDOFs, startPos, goalPos, inputSolutionFile
 * Do NOT change this. Use this to check that your solution file output
 * is verified as valid using this script.
 * */
int main(int argc, char** argv) {
    double* map;
	int x_size, y_size;

    tie(map, x_size, y_size) = loadMap(argv[1]);
	int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	string inputSolutionFile = argv[5];

    std::ifstream infile(inputSolutionFile);
    if (!infile.is_open()) {
        printf("Opening file failed! %s\n", inputSolutionFile.c_str());
        throw runtime_error("Opening file failed!");
    }
    string curLine;
    std::getline(infile, curLine); // Don't care about map
    double* curPos = nullptr;
    double* nextPos;
    while (std::getline(infile, curLine)) {
        nextPos = doubleArrayFromString(curLine);
        
        //// Check nextPos is valid
        if (!IsValidArmConfiguration(nextPos, numOfDOFs, map, x_size, y_size)) {
            infile.close();
            // printf("Did no pass verifier!\n");
            return -1; // -1 is did not pass
        }

        //// Check start positions match in the beginning
        if (curPos == nullptr) {
            if (!equalDoubleArrays(nextPos, startPos, numOfDOFs)) {
                infile.close();
                // printf("Did no pass verifier!\n");
                return -1; // -1 is did not pass
            }
        }
        curPos = nextPos;
    }
    if (!equalDoubleArrays(nextPos, goalPos, numOfDOFs)) { // Check goal positions match
        infile.close();
        // printf("Did no pass verifier!\n");
        return -1; // -1 is did not pass
    }
    infile.close();
    // printf("Passed verifier!/n");
    return 0; // 0 is success
}
