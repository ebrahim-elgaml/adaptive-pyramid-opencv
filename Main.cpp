#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include<fstream>
#include<dirent.h>
#include "Debugging.cpp"
using namespace std;
using namespace cv;

double getVariance(Mat image,int mean, vector<Point2i> neighbourPoints) {
  double resultVariance = 0;
  double variance = 0;
  for(int i =0; i<neighbourPoints.size();i++) {
    double v = image.at<uchar>(neighbourPoints[i]) - mean;
    resultVariance += v*v;
  }
  variance = resultVariance/neighbourPoints.size();
  return variance;
}
std::vector< std::vector<double> >  getCorrspondingVariance(Mat image) {
  std::vector< std::vector<double> > v;
  for(int i=0; i<image.rows; i++){
    std::vector<double> varianceVector;
    for(int j=0;j<image.cols;j++) {
      vector<Point2i> neighbourPoints = getneighbourhood(image, i, j);
      double mean = image.at<uchar>(i, j);
      double variance = getVariance(image, mean, neighbourPoints);
      // double variance = image.at<uchar>(i, j);
      varianceVector.push_back(variance);
    }
    v.push_back(varianceVector);
  }
  return v;
}
std::vector< std::vector<Node> > initNodes(Mat img) {
  std::vector< std::vector<double> > variances = getCorrspondingVariance(img);
  std::vector< std::vector<Node> > result;
  for (int i = 0; i < img.rows; i++) {
    std::vector<Node> v;
    for(int j = 0; j < img.cols; j++) {
      Node n;
      n.variance = variances[i][j];
      n.mean = img.at<uchar>(i,j);
      n.loc = Point2i(j, i);
      v.push_back(n);
    }
    result.push_back(v);
  }
  for(int i= 0; i<result.size(); i++){
    for(int j =0; j<result[i].size(); j++){
      vector<Point2i> p = getneighbourhood(img, result[i][j].loc.y, result[i][j].loc.x);
      for(int k = 0 ;k < p.size(); k++ ){
        result[i][j].neighbours.push_back(&result[p[k].y][p[k].x]);
      }
    }
  }

  return result;
}

bool continueIteration(std::vector< std::vector<Node> > nodes){
  for(int i = 0; i<nodes.size(); i++){
    for(int j = 0; j<nodes[i].size(); j++){
      if(!nodes[i][j].isMarked()) return true;
    }
  }
  return false;
}
vector< vector<Node> > getallSurvivingNodes(vector< vector<Node> > nodes){
  vector< vector<Node> > result;
  for(int i =0; i< nodes.size(); i++){
    std::vector<Node> v;
    for(int j=0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].resestNode();
        v.push_back(nodes[i][j]);
      }
    }
    result.push_back(v);
  }
  return result;
}
int countSurvivingNodes(vector< vector<Node> > nodes){
  int result;
  for(int i =0; i< nodes.size(); i++)
    for(int j=0; j<nodes[i].size(); j++)
      if(nodes[i][j].isSurvived) result++;
  return result;
}
bool shouldStop(std::vector<PyramidLevel> pyramidLevels){
  if(pyramidLevels.size() == 1) return false;
  std::cout << "HERE" << pyramidLevels.size() << '\n';

  PyramidLevel p1 = pyramidLevels[pyramidLevels.size() - 1];
  PyramidLevel p2 = pyramidLevels[pyramidLevels.size() - 2];
  std::cout << "P1: " << countSurvivingNodes(p1.nodes)<< ", P2: " << countSurvivingNodes(p2.nodes) << '\n';

  if(countSurvivingNodes(p1.nodes) == countSurvivingNodes(p2.nodes) || countSurvivingNodes(p1.nodes) == 1){
    return true;
  }
  return false;
}
void pDecide(std::vector< std::vector<Node> > &nodes ){
  while(continueIteration(nodes)) {
    for(int i = 0; i<nodes.size(); i++) {
      for(int j = 0; j<nodes[i].size(); j++) {
        nodes[i][j].decide(nodes);
      }
    }
  }
}
void pCreateLink(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        nodes[i][j].createLink(nodes);
      }
    }
  }
}
void pLinkSurvivors(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].linkSurvivors(nodes);
      }
    }
  }
}
void pDecideRoot(std::vector< std::vector<Node> > &nodes, double minContrast, double minSize, double alpha ) {
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        nodes[i][j].decideRoot(nodes, minContrast, minSize, alpha);
      }
    }
  }
}
void pUpdateMean(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].updateMean(nodes);
      }
    }
  }
}
void pUpdateVariance(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].updateVariance(nodes);
      }
    }
  }
}
void pyramidAlgorithm(Mat img, double minContrast, double minSize, double alpha) {
  vector<PyramidLevel> pyramidLevels;
  int level = 0;
  std::vector< std::vector<Node> > nodes = initNodes(img);
  PyramidLevel p = PyramidLevel(nodes);
  pyramidLevels.push_back(p);
  do {
    // std::vector< std::vector<Node> > nodes = pyramidLevels[level].nodes;
    pDecide(nodes);
    std::cout << "/* decide */" << '\n';
    if(shouldStop(pyramidLevels)) break;
    if(level == 1) break;
    pCreateLink(nodes);
    std::cout << "/* createLink */" << '\n';
    pLinkSurvivors(nodes);
    std::cout << "/* linkSurvivors */" << '\n';
    stablizeNodes(nodes);
    std::cout << "/* stablizeNodes */" << '\n';
    pDecideRoot(nodes, minContrast, minSize, alpha);
    std::cout << "/* decideRoot */" << '\n';
    removeRootNeighbours(nodes);
    std::cout << "/* removeRootNeighbours */" << '\n';
    pUpdateMean(nodes);
    std::cout << "/* updateMean */" << '\n';
    pUpdateVariance(nodes);
    std::cout << "/* updateVariance */" << '\n';
    // Test neighbours
    for(int i = 0; i < nodes.size(); i++){
      for(int j =0; j<nodes[i].size(); j++){
        if(nodes[i][j].isSurvived){
          std::cout <<  Point2i(j, i)<< " Neghbours to : ";
          for(int k =0; k<nodes[i][j].neighbours.size(); ++k){
            std::cout << nodes[i][j].neighbours[k]->loc << "-" ;
          }
          std::cout << "/* message */" << '\n';
        }
      }
    }
    std::cout << "NEWWW" << '\n';
    std::vector< std::vector<Node> > n = getallSurvivingNodes(n);
    PyramidLevel newLevel = PyramidLevel(n);
    pyramidLevels.push_back(newLevel);
    level++;
    // for(int i = 0; i < nodes.size(); i++){
    //   for(int j =0; j<nodes[i].size(); j++){
    //     if(true){
    //       std::cout <<  Point2i(j, i)<< " Neghbours to : ";
    //       for(int k =0; k<nodes[i][j].neighbours.size(); ++k){
    //         std::cout << nodes[i][j].neighbours[k]->loc << "-" ;
    //       }
    //       std::cout << "/* message */" << '\n';
    //     }
    //   }
    // }
  }while(true);
}

int main( int argc, char** argv )
{
    std::cout << "/* Hello CV */" << '\n';
    // Mat image = imread("./images/L1.jpg", 1);
    Mat image = (Mat_<uchar>(5,5) << 12, 8, 7, 3, 6, 7, 9, 4, 2, 6, 4, 6, 3, 6, 1, 9, 4, 3, 7, 4, 8, 8, 7, 6, 2);
    double minContrast = 10;
    double minSize = 4;
    double alpha = 0.3;
    pyramidAlgorithm(image, minContrast, minSize, alpha);
    std::cout << '\n';
    printf("DONE\n");
    waitKey();
    return 0;
}
