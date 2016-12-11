#include <iostream>
#include <string>
#include <math.h>
#include<fstream>
#include<dirent.h>
#include "Node.cpp"
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
