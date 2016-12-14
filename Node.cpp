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

static int currentLevel = 0;

std::vector<Point2i> getneighbourhood(Mat, int, int);
bool checkPoint(Point2i, std::vector<Point2i>);
class Node {
  public:
    bool isSurvived, isDead, isRoot;
    double variance, mean;
    Point2i loc;
    Node * bestSurvivor;
    std::vector<Node *> neighbours;
    int level;
    Node ();
    bool isMarked();
    void print();
    void decide();
    vector<Node*> getSurvivingNodes();
    void createLink();
    bool hasParent();
    void linkSurvivors();
    void addNeighbour(Node*);
    void decideRoot(double, double, double);
    int getNoOfChildren();
    void updateMean();
    void updateVariance();
    vector<Point2i> getNotRootNeighbours (std::vector< std::vector<Node> > &);
    void removeRoot();
    void resestNode();
};

Node::Node(){
  isSurvived = false;
  isDead = false;
  bestSurvivor = 0;
  isRoot = false;
  level = 0;
  mean = 0;
  variance = 0;
}
bool Node::isMarked() {
  return isSurvived || isDead;
}
void Node::decide(){
  if(isMarked()) {
    return;
  }
  for(int i = 0; i < neighbours.size(); i++) {
    if(!neighbours[i]->isMarked() && variance > neighbours[i]->variance) {
      return; //No decision
    }
  }
  // The smalles one.
  isSurvived = true;
  for(int i = 0; i < neighbours.size(); i++) {
    neighbours[i]->isDead = true;
  }
}

vector<Node*> Node::getSurvivingNodes() {
  vector<Node*> survivingNodes;
  for(int i = 0; i < neighbours.size(); i++) {
    if (neighbours[i]->isSurvived) survivingNodes.push_back(neighbours[i]);
  }
  return survivingNodes;
}
void Node::createLink() {
  vector<Node*> survivingNodes = getSurvivingNodes();
  if(survivingNodes.size() == 1) {
    this->bestSurvivor = survivingNodes[0];
    return;
  }
  double min = abs(this->mean - survivingNodes[0]->mean);
  int leastIndex = 0;
  for(int i=0; i<survivingNodes.size(); i++) {
    double diff = mean - survivingNodes[i]->mean;
    if(abs(diff) < min) {
      min = diff;
      leastIndex = i;
    }
  }
  this->bestSurvivor = survivingNodes[leastIndex];
}
void Node::linkSurvivors(){
  if(isDead) return;
  vector<Node*> points;
  for(int i =0; i < this->neighbours.size();i++) {
    if(this->neighbours[i]->bestSurvivor->loc.x == loc.x && this->neighbours[i]->bestSurvivor->loc.y == loc.y){
      std::vector<Node*> currentNeighbours =  this->neighbours[i]->neighbours;
      for(int j =0; j < currentNeighbours.size(); j++){
        if(currentNeighbours[j]->isSurvived){
          if(currentNeighbours[j]->loc.x != this->loc.x || currentNeighbours[j]->loc.y != this->loc.y){
            points.push_back(currentNeighbours[j]);
          }
        } else {
          points.push_back((currentNeighbours[j]->bestSurvivor));
        }
      }
    }
  }
  this->neighbours.clear();
  for(int i =0 ; i < points.size(); i++){
    if(!(points[i]->loc.x == this->loc.x & points[i]->loc.y == this->loc.y)) addNeighbour(points[i]);
  }
}
void Node::addNeighbour(Node *n){
  if(this->neighbours.size() == 0 ){
    if(!(n->loc.x == loc.x & n->loc.y == loc.y)) neighbours.push_back(n);
    return;
  }
  for(int i = 0 ; i < neighbours.size(); i++){
    if(neighbours[i]->loc.x == n->loc.x && neighbours[i]->loc.y == n->loc.y) return;
  }
  neighbours.push_back(n);
}
void Node::decideRoot(double minContrast, double minSize, double alpha) {
  if(isSurvived) return;
  double diff = abs(this->mean - bestSurvivor->mean);
  int x;
  if(bestSurvivor->level == currentLevel) {
    x = bestSurvivor->getNoOfChildren();
  } else {
    return;
  }
  double S;
  if(x > minSize){
    S = minContrast;
  } else {
    S = minContrast * exp(alpha*(minSize - x));
  }
  if(diff > S) {
    isRoot = true;
  }
}

int Node::getNoOfChildren() {
  if(isDead) return 0;
  int sum =0;
  for(int i = 0; i < this->neighbours.size(); i++){
    if(this->neighbours[i]->bestSurvivor == this){
      if(this->neighbours[i]->bestSurvivor->level == currentLevel) sum++;
    }
  }
  return sum;
}
void Node::updateMean() {
  double newMean = 0;
  for(int i = 0; i < this->neighbours.size(); i++) {
    newMean += neighbours[i]->mean;
  }
  newMean = newMean * 1.0/neighbours.size();
  this->mean = newMean;
}
void Node::updateVariance() {
  double rVariance = 0;
  for(int i =0;i<neighbours.size();i++) {
    double newMean = neighbours[i]->mean;
    double variance = this->mean - newMean;
    rVariance += variance*variance;
  }
  rVariance = rVariance * 1.0 /neighbours.size();
  this->variance = rVariance;
}
void Node::removeRoot(){
  if(isRoot){
    neighbours.clear();
    return;
  }
  int i = 0;
  for(i = 0; i<neighbours.size(); i++) {
    if(neighbours[i]->isRoot){
      neighbours.erase(neighbours.begin() + i);
      i=0;
    }
  }
}
void Node::resestNode(){
  isSurvived = false;
  isDead = false;
  isRoot = false;
  bestSurvivor = 0;
}
void Node::print() {
  std::cout << "(S: " << isSurvived << ", D: " << isDead << ", v: " << variance << ", m: " << mean << ", p: " << loc <<")";
}

void stablizeNodes(std::vector< std::vector<Node> > & nodes) {
  for(int i=0; i< nodes.size(); i++){
    for(int j =0; j< nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        std::vector<Node*> points = nodes[i][j].neighbours;
        for(int k = 0 ; k<points.size(); k++){
          if(points[k]->loc.x != nodes[i][j].loc.x || points[k]->loc.y != nodes[i][j].loc.y){
            nodes[i][j].neighbours[k]->addNeighbour(&nodes[i][j]);
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
void removeRootNeighbours(vector< vector<Node> > &nodes) {
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j< nodes[i].size(); j++){
      nodes[i][j].removeRoot();
    }
  }
}
bool checkPoint(Point2i p, std::vector<Point2i> v) {
  for(int i=0; i<v.size(); i++)
    if(v[i].x == p.x && v[i].y == p.y) return true;
  return false;
}
