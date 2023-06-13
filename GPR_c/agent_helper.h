#include <vector>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class Agent_helper {
public:

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  //
  //defines agent dynamics
  VectorXd forward_predict(const Ref<const VectorXd>& x_state,const Ref<const VectorXd>& velocity_vec,const  double& dt);
  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const Ref<const VectorXd>& x_state,const Ref<const VectorXd>& velocity_vec,const  double& dt);

};


