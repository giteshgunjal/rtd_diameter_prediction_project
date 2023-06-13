#include <iostream>
#include<fstream>
#include <cmath>
#include <vector>
#include <array>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues> 


#define PI 3.14159265

using namespace std;
using namespace Eigen;


class intervalGaussian 
{
public:
    // hold info about a gaussian
    struct Gauss {
        Vector2d mu;
        Matrix2d cov;
    };
    int maxSpan;
    int minSpan;

    intervalGaussian() : maxSpan{160}, minSpan{10}{};
    // vector that stores invl gaussians
    using gaussStrDataType = vector<Gauss>;

    // array of vectors that store gaussian with intervals 10ms, 20 ms,...,160ms; so 16 such vectors
    array<gaussStrDataType, 16> Store_of_Intvl_Gauss;

    MatrixXd gauss2Ellipse(const Gauss& gaussian,const int& sigma_rule, const int& num_points , const double& theta_start,const double& theta_end );

    Gauss intvlGaussApprox(const Gauss& first_gaussian, const Gauss& last_gaussian);

    void calcAllIntvlGauss(const vector<Vector3d> mus,const vector<Matrix3d> covs);

    void output_data(string& path);
    void saveTraj(string& filename, gaussStrDataType& gaussStr);


};