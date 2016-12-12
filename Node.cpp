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
bool checkPoint(Point2i, std::vector<Point2i>);
class Node {
  public:
    bool isSurvived, isDead, isRoot;
    double variance, mean;
    Point2i loc;
    Point2i bestSurvivor;
    std::vector<Point2i> neighbours;
    Node ();
    bool isMarked();
    void print();
    void decide(Mat, std::vector< std::vector<Node> > &);
    vector<Node> getSurvivingNodes(std::vector< std::vector<Node> > &);
    void createLink(Mat, std::vector< std::vector<Node> > &);
    bool hasParent();
    void linkSurvivors(std::vector< std::vector<Node> > &);
    void addNeighbour(Point2i);
    void decideRoot();
};

Node::Node(){
  isSurvived = false;
  isDead = false;
  bestSurvivor = Point2i(-1, -1);
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

vector<Node> Node::getSurvivingNodes(std::vector< std::vector<Node> > & nodes) {
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
  vector<Node> survivingNodes = getSurvivingNodes(nodes);
  Node leastNode;

  if(survivingNodes.size() == 1) {
    bestSurvivor = Point2i(survivingNodes[0].loc.x, survivingNodes[0].loc.y);
    return;
  }
  //TODO change variance with mean for testing only
  double min = abs(variance - survivingNodes[0].variance);
  leastNode = survivingNodes[0];
  for(int i=0; i<survivingNodes.size(); i++) {
    double diff = variance - survivingNodes[i].variance;
    if(abs(diff) < min) {
      min = diff;
      leastNode= survivingNodes[i];
    }
  }
  bestSurvivor = Point2i(leastNode.loc.x, leastNode.loc.y);
}
bool Node::hasParent(){
  return bestSurvivor.x != -1 && bestSurvivor.y!=-1;
}
void Node::linkSurvivors(std::vector< std::vector<Node> > & nodes){
  if(isDead) return;
  vector<Point2i> points;
  for(int i =0; i<neighbours.size();i++) {
    int x = neighbours[i].x;
    int y = neighbours[i].y;

    if(nodes[y][x].bestSurvivor.x == loc.x && nodes[y][x].bestSurvivor.y == loc.y){
      std::vector<Point2i> currentNeighbours =  nodes[y][x].neighbours;
      for(int j =0; j < currentNeighbours.size(); j++){
        Point2i p = Point2i(currentNeighbours[j].x, currentNeighbours[j].y);
        Point2i newPoint;
        if(nodes[p.y][p.x].isSurvived){
          newPoint = p;
        } else {
          newPoint = nodes[p.y][p.x].bestSurvivor;
        }
        if(newPoint.x != loc.x || newPoint.y != loc.y){
          if(!checkPoint(newPoint, points)) points.push_back(newPoint);
        }
      }
    }
  }
  neighbours.clear();
  neighbours.insert(neighbours.end(), points.begin(), points.end());
}
void Node::addNeighbour(Point2i p){
  for(int i = 0 ; i < neighbours.size(); i++){
    if(neighbours[i].x == p.x && neighbours[i].y == p.y) return;
  }
  neighbours.push_back(p);
}
void Node::decideRoot() {
  if(isSurvived) return;

}
void Node::print() {
  std::cout << "(S: " << isSurvived << ", D: " << isDead << ", v: " << variance << ", m: " << mean << ", p: " << loc <<")";
}

void stablizeNodes(std::vector< std::vector<Node> > & nodes) {
  for(int i=0; i< nodes.size(); i++){
    for(int j =0; j< nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        std::vector<Point2i> points = nodes[i][j].neighbours;
        for(int k = 0 ; k<points.size(); k++){
          if(points[k].x != nodes[i][j].loc.x || points[k].y != nodes[i][j].loc.y){
            nodes[points[k].y][points[k].x].addNeighbour(nodes[i][j].loc);
          }
        }
      }
    }
  }
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
bool checkPoint(Point2i p, std::vector<Point2i> v) {
  for(int i=0; i<v.size(); i++)
    if(v[i].x == p.x && v[i].y == p.y) return true;
  return false;
}
