#include "quintic_walk/fuzzy.hpp"

//    -------------------------------------
//    |  X  |  VN |  N  |  Z  |  P  |  VP |
//    -------------------------------------
//    |  VN |  VP | VP  |  P  |  N  |  N  |     VN  = Very Negetive = 0
//    -------------------------------------
//    |  N  |  VP |  P  |  P  |  N  |  N  |     N   = Negetive      = 1
//    -------------------------------------
//    |  Z  |  P  |  P  |  Z  |  N  |  N  |     Z  = Zero           = 2
//    -------------------------------------
//    |  P  |  P  |  P  |  N  |  N  |  VN |     P  = Positive       = 3
//    -------------------------------------
//    |  VP |  P  |  P  |  N  |  VN |  VN |     VP = Very Positive  = 4
//    -------------------------------------

void Fuzzy::setLimit(double L0, double L1, double L2, double L3, double L4)
{
  LIMIT[0] = L0;
  LIMIT[1] = L1;
  LIMIT[2] = L2;
  LIMIT[3] = L3;
  LIMIT[4] = L4;
}


Eigen::VectorXd Fuzzy::predicate(double input)
{
  Eigen::VectorXd Value(5);
  Value.setZero();

  if(input <= this->LIMIT[0])      Value(0) = 1;
  else if(input == this->LIMIT[1]) Value(1) = 1;
  else if(input == this->LIMIT[2]) Value(2) = 1;
  else if(input == this->LIMIT[3]) Value(3) = 1;
  else if(input >= this->LIMIT[4]) Value(4) = 1;
  else if(input > this->LIMIT[0] && input < this->LIMIT[1])
  {
    Value(0) = (this->LIMIT[1] - input)/(this->LIMIT[1] - this->LIMIT[0]);
    Value(1) = (input - this->LIMIT[0])/(this->LIMIT[1] - this->LIMIT[0]);
  }
  else if(input > this->LIMIT[1] && input < this->LIMIT[2])
  {
    Value(1) = (this->LIMIT[2] - input)/(this->LIMIT[2] - this->LIMIT[1]);
    Value(2) = (input - this->LIMIT[1])/(this->LIMIT[2] - this->LIMIT[1]);
  }
  else if(input > this->LIMIT[2] && input < this->LIMIT[3])
  {
    Value(2) = (this->LIMIT[3] - input)/(this->LIMIT[3] - this->LIMIT[2]);
    Value(3) = (input - this->LIMIT[2])/(this->LIMIT[3] - this->LIMIT[2]);
  }
  else if(input > this->LIMIT[3] && input < this->LIMIT[4])
  {
    Value(3) = (this->LIMIT[4] - input)/(this->LIMIT[4] - this->LIMIT[3]);
    Value(4) = (input - this->LIMIT[3])/(this->LIMIT[4] - this->LIMIT[3]);
  }

return Value;
}

Eigen::VectorXd Fuzzy::membership(const Eigen::VectorXd& C1, const Eigen::VectorXd& C2)
{
  Eigen::VectorXd inferenceOut(5);
  inferenceOut.setZero();
  int k;

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      k = rule[i][j];
      inferenceOut(k) = std::fmax(inferenceOut(k), std::fmin(C1(i), C2(j)));

    }
  }

  return inferenceOut;
}

Eigen::VectorXd Fuzzy::membershipVal(const Eigen::VectorXd& predicate)
{
  Eigen::VectorXd predicateOut(5);
  predicateOut.setZero();

  predicateOut(0) = ((1 - predicate(0)) * this->LIMIT[1] + predicate(0) * this->LIMIT[0]);
  predicateOut(1) = ((1 - predicate(1)) * this->LIMIT[2] + predicate(1) * this->LIMIT[1]) - (predicate(1) * this->LIMIT[1] - (predicate(1) - 1) * this->LIMIT[0]) / 2;
  predicateOut(2) = ((predicate(2) * this->LIMIT[2] - (predicate(2) - 1) * this->LIMIT[1]) - (1 - predicate(2)) * this->LIMIT[3] + predicate(2) * this->LIMIT[2]) / 2;
  predicateOut(3) = ((1 - predicate(3)) * this->LIMIT[4] + predicate(3) * this->LIMIT[3]) - (predicate(3) * this->LIMIT[3] - (predicate(3) - 1) * this->LIMIT[2]) / 2;
  predicateOut(4) = (predicate(4) * this->LIMIT[4] - (predicate(4) - 1) * this->LIMIT[3]);

  return predicateOut;
}

double Fuzzy::outputMoM(const Eigen::VectorXd& membership)
{
  double temp = .0;
  int flag = 0;
  double Output = .0;
  bool max = false;

  for (int i = 0; i < 5; i++) {
    if(membership(i) > temp)
    {
      temp = membership(i);
      flag = i;
    }
    if(membership(i) == 1)
      max = true;
  }

  if(max)
    return this->LIMIT[flag];
  else
  {
    if(flag == 0)
      return Output = ((1 - membership(0)) * this->LIMIT[1] + membership(0) * this->LIMIT[0] + this->LIMIT[0])/2;
    else if(flag == 2)
      return Output = ((membership(flag) * this->LIMIT[flag] - (membership(flag) - 1) * this->LIMIT[flag - 1] + (1 - membership(flag)) * this->LIMIT[flag + 1] + membership(flag) * this->LIMIT[flag])) / 2;
    else if(flag == 4)
      return Output = ((membership(4) * this->LIMIT[4] - (membership(4) - 1) * this->LIMIT[3]) + this->LIMIT[4])/2;
    else
      return Output = ((1 - membership(flag)) * this->LIMIT[flag + 1] + membership(flag) * this->LIMIT[flag]) + (membership(flag) * this->LIMIT[flag] - (membership(flag) - 1) * this->LIMIT[flag - 1]) / 2;
  }
}

double Fuzzy::outputWeightAvg(const Eigen::VectorXd& membership, const Eigen::VectorXd& membershipVal)
{
  double pemb   = .0;
  double peny   = .0;
  double output = .0;
  for (int i = 0; i < 5; i++) {
    pemb += membership(i) * membershipVal(i);
    peny += membership(i);
  }
  // std::cout << pemb << '\t';
  // std::cout << peny << '\n';

  return output = pemb/peny;
}
