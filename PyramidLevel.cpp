#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include<fstream>
#include<dirent.h>
#include "Node.cpp"
using namespace std;
using namespace cv;
class PyramidLevel {
public:
  std::vector< std::vector<Node> >nodes;
  PyramidLevel (std::vector< std::vector<Node> >);
};
PyramidLevel::PyramidLevel(std::vector< std::vector<Node> > barbory){
  nodes = barbory;
}
