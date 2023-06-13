#include <iostream>
#include <cmath>
#include "agent_helper.h"

using namespace std;
using namespace Eigen;
using std::vector;


VectorXd Agent_helper::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Validate the estimations vector
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    cout<<"Error in size of Estimations vector or size mismatch with Ground Truth vector";
    return rmse;
  }
  
  //Accumulate the residual
  for(int i = 0; i < estimations.size(); ++i){
  VectorXd residual = estimations[i] - ground_truth[i];
  rmse = rmse + (residual.array() * residual.array()).matrix();
  }

  //Mean and Sqrt the error
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

VectorXd Agent_helper::forward_predict(const Ref<const VectorXd>& x_state,const Ref<const VectorXd>& velocity_vec,const  double& dt){
  // cout<<velocity_vec(0)<< ", " <<velocity_vec(1) <<endl;
  Vector3d next_state = {(x_state(0) + velocity_vec(0)*cos(x_state(2))*dt),  (x_state(1) + velocity_vec(0)*sin(x_state(2))*dt), (x_state(2) + velocity_vec(1)*dt)};
  return next_state;

}

MatrixXd Agent_helper::CalculateJacobian(const Ref<const VectorXd>& x_state,const Ref<const VectorXd>& velocity_vec, const double &dt) {

  //Initalize the Jacobian
  MatrixXd Hj(3,3);
  //Retrive the state values from the vector

  float h = x_state(2);
  float v = velocity_vec(0);

  Hj(0,0) = 1;
  Hj(0,1) = 0;
  Hj(0,2) = - v* sin(h)*dt;
  Hj(1,0) = 0;
  Hj(1,1) = 1;
  Hj(1,2) = v* cos(h)*dt;
  Hj(2,0) = 0;
  Hj(2,1) = 0;
  Hj(2,2) = 1;
  return Hj;

}
