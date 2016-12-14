#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include<fstream>
#include<dirent.h>
#include "PyramidLevel.cpp"
using namespace std;

void printVectorOfVectores(std::vector< std::vector<double> > v) {
  for( int i = 0; i< v.size(); ++i){
    for (int j = 0; j < v[i].size(); j++) {
      std::cout << "" << v[i][j] << ", ";
    }
    std::cout << '\n';
  }
  std::cout << '\n';
}

void printVectorOfVectores(std::vector< std::vector<Node> > v) {
  for( int i = 0; i< v.size(); ++i){
    for (int j = 0; j < v[i].size(); j++) {
      v[i][j].print();
    }
    std::cout << '\n';
  }
  std::cout << '\n';
}

void printMeans(std::vector< std::vector<Node> > v) {
  for( int i = 0; i< v.size(); ++i){
    for (int j = 0; j < v[i].size(); j++) {
      std::cout << v[i][j].mean << " ";
    }
    std::cout << '\n';
  }
  std::cout << '\n';
}
