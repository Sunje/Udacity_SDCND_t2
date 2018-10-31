#include <iostream>
#include "tools.h"
#include <fstream>
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {

}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
  
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;

  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
    
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
  
}

void Tools::StoreNIS(MeasurementPackage meas_package, const double NIS){

  std::cout << "This function is running... and NIS value is " << NIS << std::endl;
  std::cout << "But WHY it doesn't store the value as I expected!!!" << std::endl;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR){

    if (!radarfile.is_open()){
      radarfile.open ("radar.dat");
      radarfile<<NIS<<"\n";
    }
    else if (radarfile.is_open()){
      radarfile<<NIS<<"\n";
    }

  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER){

    if (!lidarfile.is_open()){
      lidarfile.open ("laser.dat");
      lidarfile<<NIS<<"\n";
    }
    else if (lidarfile.is_open()){
      lidarfile<<NIS<<"\n";
    }

  }
  return;
  
}