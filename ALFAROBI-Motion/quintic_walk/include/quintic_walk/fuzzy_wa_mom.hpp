#ifndef FUZZY_HPP
#define FUZZY_HPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Eigen>

//    -------------------------------------
//    |  X  |  VN |  N  |  Z  |  P  |  VP |
//    -------------------------------------
//    |  VN |  VP | VP  |  P  |  -  |  -  |     VN  = Very Negetive = 0
//    -------------------------------------
//    |  N  |  VP |  P  |  P  |  -  |  -  |     N   = Negetive      = 1
//    -------------------------------------
//    |  Z  |  P  |  P  |  Z  |  N  |  N  |     Z  = Zero           = 2
//    -------------------------------------
//    |  P  |  -  |  -  |  N  |  N  |  VN |     P  = Positive       = 3
//    -------------------------------------
//    |  VP |  -  |  -  |  N  |  VN |  VN |     VP = Very Positive  = 4
//    -------------------------------------

class Fuzzy{
private:
  double LIMIT[5];
  int rule[5][5] = {{4, 4, 3, 3, 4},
                    {4, 3, 3, 4, 4},
                    {3, 3, 2, 1, 1},
                    {0, 0, 1, 1, 0},
                    {0, 1, 1, 0, 0}};

public:
    void setLimit(double L0, double L1, double L2, double L3, double L4);
    Eigen::VectorXd predicate(double input);
    Eigen::VectorXd membership(const Eigen::VectorXd& C1, const Eigen::VectorXd& C2);
    Eigen::VectorXd membershipVal(const Eigen::VectorXd& predicate);
    double outputMoM(const Eigen::VectorXd& membership);
    double outputWeightAvg(const Eigen::VectorXd& membership, const Eigen::VectorXd& membershipVal);
};

#endif
