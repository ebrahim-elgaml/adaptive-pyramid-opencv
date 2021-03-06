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

int myRound(double d) {
    int x = (int)d;
    return (d-x) > 0.4 ? ceil(d) : x;
}
double getVariance(Mat image,int mean, vector<Point2i> neighbourPoints) {
  double resultVariance = 0;
  double variance = 0;
  for(int i =0; i<neighbourPoints.size();i++) {
    double v = image.at<uchar>(neighbourPoints[i]) - mean;
    resultVariance += v*v;
  }
  variance = resultVariance * 1.0/neighbourPoints.size();
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
      n.mean = img.at<uchar>(i,j) * 1.0;
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
int countSurvivingNodes(vector< vector<Node> > nodes){
  int result;
  for(int i =0; i< nodes.size(); i++)
    for(int j=0; j<nodes[i].size(); j++)
      if(nodes[i][j].isSurvived) result++;
  return result;
}
void createNewLevel(vector< vector<Node> > &nodes){
  for(int i =0; i< nodes.size(); i++){
    for(int j=0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].resestNode();
        nodes[i][j].level++;
      }
    }
  }
  currentLevel++;
}

bool shouldStop(vector< vector<Node> > nodes, int oldCount){
  if(currentLevel == 0) return false;
  if(oldCount == countSurvivingNodes(nodes) || countSurvivingNodes(nodes) == 1){
    return true;
  }
  return false;
}
void pDecide(std::vector< std::vector<Node> > &nodes ){
  while(continueIteration(nodes)) {
    for(int i = 0; i<nodes.size(); i++) {
      for(int j = 0; j<nodes[i].size(); j++) {
        if(nodes[i][j].level == currentLevel) nodes[i][j].decide();
      }
    }
  }
}
void pCreateLink(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        if(nodes[i][j].level == currentLevel) nodes[i][j].createLink();
      }
    }
  }
}
void pLinkSurvivors(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        if(nodes[i][j].level == currentLevel) nodes[i][j].linkSurvivors();
      }
    }
  }
}
void pDecideRoot(std::vector< std::vector<Node> > &nodes, double minContrast, double minSize, double alpha ) {
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        if(nodes[i][j].level == currentLevel) nodes[i][j].decideRoot(minContrast, minSize, alpha);
      }
    }
  }
}
void pUpdateMean(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        if(nodes[i][j].level == currentLevel) nodes[i][j].updateMean();
      }
    }
  }
}
void pUpdateVariance(std::vector< std::vector<Node> > &nodes ){
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        if(nodes[i][j].level == currentLevel) nodes[i][j].updateVariance();
      }
    }
  }
}
void createNewImage(Mat img, std::vector< std::vector<Node> > &nodes) {
  Mat result = Mat::zeros( img.size(), img.type() );
  for(int i = 0; i < result.rows; i++)
    for(int j = 0; j < result.cols; j++)
      result.at<uchar>(i, j) = (uchar)myRound(nodes[i][j].mean);
  imwrite("./output/out.png", result);
  namedWindow( "New window", WINDOW_AUTOSIZE );
  imshow( "New window", result );
  namedWindow( "Old window", WINDOW_AUTOSIZE );
  imshow( "Old window", img );
}
void pyramidAlgorithm(Mat img, double minContrast, double minSize, double alpha) {
  std::vector< std::vector<Node> > nodes = initNodes(img);
  int oldCount = 0;
  do {
    pDecide(nodes);
    std::cout << "/* decide */ Level : " <<  currentLevel  << '\n';
    if(shouldStop(nodes, oldCount)) break;
    pCreateLink(nodes);
    std::cout << "/* createLink */ Level : " <<  currentLevel  << '\n';
    pLinkSurvivors(nodes);
    std::cout << "/* linkSurvivors */ Level : " <<  currentLevel  << '\n';
    stablizeNodes(nodes);
    std::cout << "/* stablizeNodes */ Level : " <<  currentLevel  << '\n';
    pDecideRoot(nodes, minContrast, minSize, alpha);
    std::cout << "/* decideRoot */ Level : " <<  currentLevel  << '\n';
    removeRootNeighbours(nodes);
    std::cout << "/* removeRootNeighbours */ Level : " <<  currentLevel  << '\n';
    pUpdateMean(nodes);
    std::cout << "/* updateMean */ Level : " <<  currentLevel <<'\n';
    pUpdateVariance(nodes);
    std::cout << "/* updateVariance */ Level : " <<  currentLevel << '\n';
    // Test neighbours
    // for(int i = 0; i < nodes.size(); i++){
    //   for(int j =0; j<nodes[i].size(); j++){
    //     std::cout << nodes[i][j].mean << " ";
    //     // if(nodes[i][j].isSurvived){
    //     //   std::cout <<  Point2i(j, i)<< " Neghbours to : ";
    //     //   for(int k =0; k<nodes[i][j].neighbours.size(); ++k){
    //     //     std::cout << nodes[i][j].neighbours[k]->loc << "-" ;
    //     //   }
    //     //   std::cout << "/* message */" << '\n';
    //     // }
    //   }
    //   std::cout << "/* message */" << '\n';
    // }
    oldCount = countSurvivingNodes(nodes);
    createNewLevel(nodes);
  }while(true);
  createNewImage(img, nodes);
}

int main( int argc, char** argv )
{
    std::cout << "/* Hello CV */" << '\n';
    Mat image = imread("./images/lec.png", 0);
    // Mat image = (Mat_<uchar>(5,5) << 12, 8, 7, 3, 6, 7, 9, 4, 2, 6, 4, 6, 3, 6, 1, 9, 4, 3, 7, 4, 8, 8, 7, 6, 2);
    double minContrast = 10;
    double minSize = 4;
    double alpha = 0.3;
    pyramidAlgorithm(image, minContrast, minSize, alpha);
    std::cout << '\n';
    printf("DONE\n");
    waitKey(0);
    return 0;
}
