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

std::vector<Point2i> getneighbourhood(Mat image, int y, int x){
  vector<Point2i> neighbourPoints;
  double x1 = x-1;
  double x2 = x+1;
  double y1 = y-1;
  double y2 = y+1;
  if(x1 >= 0 && y1 >= 0){
    neighbourPoints.push_back(Point2i(x1,y1));
  }
  if(y1 >=0) {
    neighbourPoints.push_back(Point2i(x,y1));
  }
  if(x2 < image.cols && y1 >=0) {
    neighbourPoints.push_back(Point2i(x2,y1));
  }
  if(x1 >= 0) {
    neighbourPoints.push_back(Point2i(x1,y));
  }
  if(x2 < image.cols) {
    neighbourPoints.push_back(Point2i(x2,y));
  }
  if(x1 >=0 && y2 < image.rows) {
    neighbourPoints.push_back(Point2i(x1,y2));
  }
  if(y2 < image.rows) {
    neighbourPoints.push_back(Point2i(x,y2));
  }
  if(x2 < image.cols && y2 < image.rows) {
    neighbourPoints.push_back(Point2i(x2,y2));
  }
    return neighbourPoints;
}
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
  for(int i=0;i<image.rows;i++){
    std::vector<double> varianceVector;
    for(int j=0;j<image.cols;j++) {
      vector<Point2i> neighbourPoints= getneighbourhood(image, i,j);
      double mean= getMean(image,neighbourPoints);
      double variance = getVariance(image,mean,neighbourPoints);
      // std::cout << "/* My Mean */" << mean << " myVariance: " << variance << '\n';
      varianceVector.push_back(variance);
    }
    v.push_back(varianceVector);
  }
  return v;
}
std::vector< std::vector<double> >  getCorrspondingMean(Mat image) {
  std::vector< std::vector<double> > m;
  for(int i=0;i<image.rows;i++){
    std::vector< std::vector<double> > meanVector;
    for(int j=0;j<image.cols;j++) {
      vector<Point2i> neighbourPoints= getneighbourhood(image, i,j);
      double mean= getMean(image,neighbourPoints);
      meanVector.push_back(mean);
    }
      m.push_back(meanVector);
  }
  return m;
}

void printVectorOfVectores(std::vector< std::vector<double> > v) {
  for( int i = 0; i< v.size(); ++i){
    for (int j = 0; j < v[i].size(); j++) {
      std::cout << "" << v[i][j] << ", ";
    }
    std::cout << '\n';
  }
}
int main( int argc, char** argv )
{
    std::cout << "/* Hello CV */" << '\n';

    printf("DONE\n");
    // Mat image = imread("./images/L1.jpg", 1);
    Mat image = (Mat_<uchar>(5,5) << 42, 43, 43, 44, 45, 43, 43, 44, 45, 45, 44, 44, 45, 46, 46, 44, 45, 46, 46, 47, 45, 46, 46, 47, 48);
    printVectorOfVectores(getCorrspondingVariance(image));

    waitKey();
    return 0;
}
