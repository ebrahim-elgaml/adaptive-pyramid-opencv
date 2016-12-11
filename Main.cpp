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

double getMean(Mat image,vector<Point2i> neighbourPoints) {
  double sum =0;
  for(int i=0;i<neighbourPoints.size();i++){
    sum+= image.at<uchar>(neighbourPoints[i]);
  }
  double mean = sum/neighbourPoints.size();
  return mean;
}

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
      double mean= getMean(image, neighbourPoints);
      double variance = getVariance(image, mean, neighbourPoints);
      varianceVector.push_back(variance);
    }
    v.push_back(varianceVector);
  }
  return v;
}
std::vector< std::vector<double> >  getCorrspondingMean(Mat image) {
  std::vector< std::vector<double> > m;
  for(int i=0; i<image.rows; i++){
    std::vector<double> meanVector;
    for(int j=0;j<image.cols;j++) {
      vector<Point2i> neighbourPoints= getneighbourhood(image, i,j);
      double mean= getMean(image, neighbourPoints);
      meanVector.push_back(mean);
    }
      m.push_back(meanVector);
  }
  return m;
}

std::vector< std::vector<Node> > initNodes(Mat img) {
  std::vector< std::vector<double> > variances = getCorrspondingVariance(img);
  std::vector< std::vector<double> > means = getCorrspondingMean(img);
  std::vector< std::vector<Node> > result;
  for (int i = 0; i < img.rows; i++) {
    std::vector<Node> v;
    for(int j = 0; j < img.cols; j++) {
      Node n;
      n.variance = variances[i][j];
      n.mean = means[i][j];
      n.x = j; n.y = i;
      v.push_back(n);
    }
    result.push_back(v);
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
void pyramidAlgorithm(Mat img) {
  std::vector< std::vector<Node> > nodes = initNodes(img);
  printVectorOfVectores(nodes);
  std::cout << "/* message */" << continueIteration(nodes) << '\n';
  while(continueIteration(nodes)) {
    for(int i = 0; i<nodes.size(); i++){
      for(int j = 0; j<nodes[i].size(); j++){
        nodes[i][j].decide(img, nodes);
      }
    }
  }
  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].isDead){
        nodes[i][j].createLink(img, nodes);
      }
    }
  }

  for(int i = 0; i < nodes.size(); i++){
    for(int j =0; j<nodes[i].size(); j++){
      if(nodes[i][j].hasParent()){
        std::cout << "/*PARENT */" << '\n';
        Point2i p = nodes[i][j].parentNodePoint;
        nodes[p.y][p.x].print();
        // std::cout << "/* message */" <<  << '\n';
        // std::cout << nodes[i][j].parentNode << '\n';
        std::cout << "/*PARENT */" << '\n';

      }
    }
  }

  std::cout << "/* message */" << continueIteration(nodes) << '\n';

  // nodes[0][0].decide(img, nodes);
  printVectorOfVectores(nodes);

  std::cout << "/* message */" << nodes[4][4].x << '\n';
}

int main( int argc, char** argv )
{
    std::cout << "/* Hello CV */" << '\n';
    // Mat image = imread("./images/L1.jpg", 1);
    Mat image = (Mat_<uchar>(5,5) << 42, 43, 43, 44, 45, 43, 43, 44, 45, 45, 44, 44, 45, 46, 46, 44, 45, 46, 46, 47, 45, 46, 46, 47, 48);
    pyramidAlgorithm(image);
    std::cout << '\n';
    printf("DONE\n");
    waitKey();
    return 0;
}
