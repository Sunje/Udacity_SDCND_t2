#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include <fstream>
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:

  std::ofstream lidarfile;
  std::ofstream radarfile;
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  void StoreNIS(MeasurementPackage meas_package, const double NIS);

};

#endif /* TOOLS_H_ */