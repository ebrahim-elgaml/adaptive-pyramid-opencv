#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include<fstream>
#include<dirent.h>
using namespace std;
using namespace cv;

std::vector<Point2i> getneighbourhood(Mat, int, int);

class Node {
  public:
    bool isSurvived, isDead;
    double variance, mean;
    int x, y;// x colsm y rows
    Node ();
    bool isMarked();
    void print();
    void decide(Mat, std::vector< std::vector<Node> > &);
};

Node::Node(){
  isSurvived = false;
  isDead = false;
}
bool Node::isMarked() {
  return isSurvived || isDead;
}
void Node::decide(Mat img, std::vector< std::vector<Node> > &nodes){
  vector<Point2i> neighbourPoints = getneighbourhood(img, y, x);
  bool isLessThanAll;
  double minVariance = variance;
  int minIndex;
  if(isMarked()) {
    return;
  }
  for(int i = 0; i < neighbourPoints.size(); i++) {
    int x = neighbourPoints[i].x;
    int y = neighbourPoints[i].y;
    if(!nodes[y][x].isMarked() && variance > nodes[y][x].variance) {
      return; //No decision
    }
  }
  // The smalles one.
  isSurvived = true;
  for(int i = 0; i < neighbourPoints.size(); i++) {
    int x = neighbourPoints[i].x;
    int y = neighbourPoints[i].y;
    nodes[y][x].isDead = true;
  }
}
void Node::print() {
  std::cout << "(isSurvived: " << isSurvived << ", isDead: " << isDead << ", v: " << variance << ", m: " << mean << "At x: " << x << ",y: " << y <<")";
}
std::vector<Point2i> getneighbourhood(Mat image, int y, int x){
  vector<Point2i> neighbourPoints;
  double x1 = x-1;double x2 = x+1;double y1 = y-1;double y2 = y+1;
  if(x1 >= 0 && y1 >= 0) neighbourPoints.push_back(Point2i(x1,y1));
  if(y1 >=0) neighbourPoints.push_back(Point2i(x,y1));
  if(x2 < image.cols && y1 >=0) neighbourPoints.push_back(Point2i(x2,y1));
  if(x1 >= 0) neighbourPoints.push_back(Point2i(x1,y));
  if(x2 < image.cols) neighbourPoints.push_back(Point2i(x2,y));
  if(x1 >=0 && y2 < image.rows) neighbourPoints.push_back(Point2i(x1,y2));
  if(y2 < image.rows) neighbourPoints.push_back(Point2i(x,y2));
  if(x2 < image.cols && y2 < image.rows) neighbourPoints.push_back(Point2i(x2,y2));
  return neighbourPoints;
}
