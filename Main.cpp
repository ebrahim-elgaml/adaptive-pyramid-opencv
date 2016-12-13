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
void pyramidAlgorithm(Mat img, double minContrast, double minSize, double alpha) {
  std::vector< std::vector<Node> > nodes = initNodes(img);
  while(continueIteration(nodes)) {
    for(int i = 0; i<nodes.size(); i++){
      for(int j = 0; j<nodes[i].size(); j++){
        nodes[i][j].decide(img, nodes);
      }
    }
  }
  std::cout << "/* message */" << '\n';
  // printMeans(nodes);
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        nodes[i][j].createLink(img, nodes);
      }
    }
  }
  std::cout << "/* createLink */" << '\n';
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].linkSurvivors(nodes);
      }
    }
  }
  std::cout << "/* linkSurvivors */" << '\n';
  stablizeNodes(nodes);
  std::cout << "/* stablizeNodes */" << '\n';
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        nodes[i][j].decideRoot(nodes, minContrast, minSize, alpha);
      }
    }
  }

  std::cout << "/* decideRoot */" << '\n';

  removeRootNeighbours(nodes);
  std::cout << "/* removeRootNeighbours */" << '\n';

  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].updateMean(nodes);
      }
    }
  }
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isSurvived){
        nodes[i][j].updateVariance(nodes);
      }
    }
  }
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
