// To test the parallel parking controller before implementing it in a ROS pkg

#include <cmath.h>
#include <stdio.h>

using namespace std;

#define steeringGain 2
#define throttleGain 2

float computeDistance(float first, float second);

int main()
{
  x = -2;
  xd = -5;
  while(x != xd){
      steeringCtrl = -steeringGain*(xd-x);
      
  }

}

float computeDistance(float first, float second)
{
    return first - second;
}
