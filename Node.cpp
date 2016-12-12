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
    int x, y;// x cols, y rows
    Point2i parentNodePoint;
    std::vector<Point2i> neighbours;
    Node ();
    bool isMarked();
    void print();
    void decide(Mat, std::vector< std::vector<Node> > &);
    vector<Node> getSurvivingNodes(Mat, std::vector< std::vector<Node> > &);
    void createLink(Mat, std::vector< std::vector<Node> > &);
    bool hasParent();
};

Node::Node(){
  isSurvived = false;
  isDead = false;
  parentNodePoint = Point2i(-1, -1);
}
bool Node::isMarked() {
  return isSurvived || isDead;
}
void Node::decide(Mat img, std::vector< std::vector<Node> > &nodes){
  vector<Point2i> neighbourPoints = neighbours;
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

vector<Node> Node::getSurvivingNodes(Mat img, std::vector< std::vector<Node> > & nodes) {
  vector<Point2i> neighbourPoints = neighbours;
  vector<Node> survivingNodes;
  for(int i = 0; i < neighbourPoints.size(); i++) {
    int x = neighbourPoints[i].x;
    int y = neighbourPoints[i].y;
    if (nodes[y][x].isSurvived) survivingNodes.push_back(nodes[y][x]);
  }
  return survivingNodes;
}
void Node::createLink(Mat img, std::vector< std::vector<Node> > & nodes) {
  vector<Node> survivingNodes = getSurvivingNodes(img,nodes);
  Node leastNode;

  if(survivingNodes.size() == 1) {
    parentNodePoint = Point2i(survivingNodes[0].x, survivingNodes[0].y);
    return;
  }
  double min = abs(mean - survivingNodes[0].mean);

  for(int i=0; i<survivingNodes.size(); i++) {
    double diff = mean - survivingNodes[i].mean;
    if(abs(diff) < min) {
      min = diff;
      leastNode= survivingNodes[i];
    }
  }
  if(min != -1) parentNodePoint = Point2i(leastNode.x, leastNode.y);
}
bool Node::hasParent(){
  return parentNodePoint.x != -1 && parentNodePoint.y!=-1;
}
void Node::print() {
  // std::cout << "(At x: " << x << ",y: " << y <<")";

  std::cout << "(S: " << isSurvived << ", D: " << isDead << ", v: " << variance << ", m: " << mean << ", x: " << x << ",y: " << y <<")";
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
