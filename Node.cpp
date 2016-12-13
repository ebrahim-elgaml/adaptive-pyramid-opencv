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
    Node * bestSurvivor;
    std::vector<Node *> neighbours;
    Node ();
    bool isMarked();
    void print();
    void decide(Mat, std::vector< std::vector<Node> > &);
    vector<Node*> getSurvivingNodes(std::vector< std::vector<Node> > &);
    void createLink(Mat, std::vector< std::vector<Node> > &);
    bool hasParent();
    void linkSurvivors(std::vector< std::vector<Node> > &);
    void addNeighbour(Node*);
    void decideRoot(vector< vector<Node> > &, double, double, double);
    int getNoOfChildren(vector< vector<Node> > &);
    void updateMean(std::vector< std::vector<Node> > &);
    void updateVariance(std::vector< std::vector<Node> > &);
    vector<Point2i> getNotRootNeighbours (std::vector< std::vector<Node> > &);
    void removeRoot();
};

Node::Node(){
  isSurvived = false;
  isDead = false;
  bestSurvivor = 0;
  isRoot = false;
}
bool Node::isMarked() {
  return isSurvived || isDead;
}
void Node::decide(Mat img, std::vector< std::vector<Node> > &nodes){
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

vector<Node*> Node::getSurvivingNodes(std::vector< std::vector<Node> > & nodes) {
  vector<Node*> survivingNodes;
  for(int i = 0; i < neighbours.size(); i++) {
    if (neighbours[i]->isSurvived) survivingNodes.push_back(neighbours[i]);
  }
  return survivingNodes;
}
void Node::createLink(Mat img, std::vector< std::vector<Node> > & nodes) {
  vector<Node*> survivingNodes = getSurvivingNodes(nodes);
  if(survivingNodes.size() == 1) {
    bestSurvivor = survivingNodes[0];
    return;
  }
  double min = abs(variance - survivingNodes[0]->mean);
  int leastIndex = 0;
  for(int i=0; i<survivingNodes.size(); i++) {
    double diff = mean - survivingNodes[i]->mean;
    if(abs(diff) < min) {
      min = diff;
      leastIndex = i;
    }
  }
  bestSurvivor = survivingNodes[leastIndex];
}
void Node::linkSurvivors(std::vector< std::vector<Node> > & nodes){
  if(isDead) return;
  vector<Node*> points;
  for(int i =0; i<neighbours.size();i++) {
    if(neighbours[i]->bestSurvivor->loc.x == loc.x && neighbours[i]->bestSurvivor->loc.y == loc.y){
      std::vector<Node*> currentNeighbours =  neighbours[i]->neighbours;
      for(int j =0; j < currentNeighbours.size(); j++){
        if(currentNeighbours[j]->isSurvived){
          if(currentNeighbours[j]->loc.x != loc.x || currentNeighbours[j]->loc.y != loc.y){
            points.push_back(currentNeighbours[j]);
          }
        } else {
          points.push_back((currentNeighbours[j]->bestSurvivor));
        }
      }
    }
  }
  if(points.size() > 0) neighbours.clear();
  for(int i =0 ; i < points.size(); i++){
    addNeighbour(points[i]);
  }
}
void Node::addNeighbour(Node *n){
  if(neighbours.size() == 0 ){
    neighbours.push_back(n);
    return;
  }
  for(int i = 0 ; i < neighbours.size(); i++){
    if(neighbours[i]->loc.x == n->loc.x && neighbours[i]->loc.y == n->loc.y) return;
  }
  neighbours.push_back(n);
}
void Node::decideRoot(vector< vector<Node> > & nodes, double minContrast, double minSize, double alpha) {
  if(isSurvived) return;
  double diff = abs(mean - bestSurvivor->mean);
  int x = bestSurvivor->getNoOfChildren(nodes);
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

int Node::getNoOfChildren(vector< vector<Node> > & nodes) {
  if(isDead) return 0;
  int sum =0;
  for(int i =0; i  < nodes.size(); i++){
    for(int j =0; j <nodes[i].size(); j++){
      if(nodes[i][j].isDead && nodes[i][j].bestSurvivor->loc.x == loc.x && nodes[i][j].bestSurvivor->loc.y == loc.y){
        sum++;
      }
    }
  }
  return sum;
}
void Node::updateMean(std::vector< std::vector<Node> > & nodes) {
  double mean = 0;
  double result = 0;
  for(int i = 0; i<neighbours.size();i++) {
    mean += neighbours[i]->mean;
  }
  result = mean/neighbours.size();
  mean = result;
}
void Node::updateVariance(std::vector< std::vector<Node> > & nodes) {
  double rVariance = 0;
  double resVariance = 0;
  for(int i =0;i<neighbours.size();i++) {
    double newMean = neighbours[i]->mean;
    double variance = mean - newMean;
    rVariance+= variance*variance;
  }
  resVariance = rVariance/neighbours.size();
  variance = resVariance;
}
void Node::removeRoot(){
  std::cout << loc << '\n';

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
