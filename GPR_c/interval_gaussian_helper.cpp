#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include "interval_gaussian_helper.h"


#define PI 3.14159265

using namespace std;
using namespace Eigen;

MatrixXd intervalGaussian::gauss2Ellipse(const Gauss& gaussian,const int& sigma_rule, const int& num_points , const double& theta_start,const double& theta_end )
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(gaussian.cov);
    if (eigensolver.info() != Eigen::Success) abort();
    Matrix2d eigVal = eigensolver.eigenvalues().array().sqrt().matrix().asDiagonal();
    Matrix2d eigVec = eigensolver.eigenvectors();
    VectorXd theta = VectorXd::LinSpaced(num_points, theta_start,theta_end);
    MatrixXd circlePts;
    circlePts.resize(num_points, 2);
    circlePts.col(0) =  theta.array().cos();
    circlePts.col(1) =  theta.array().sin();
    MatrixXd elliPts = circlePts*eigVal*(eigVec.transpose());
    elliPts.transpose().colwise()+= gaussian.mu;

    return elliPts;

};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// This can be optimized later if necessary
intervalGaussian::Gauss intervalGaussian::intvlGaussApprox(const Gauss& first_gaussian, const Gauss& last_gaussian)
{   double confidence_scale = 6;
    int sigma_rule =3;
    intervalGaussian::Gauss theBigOne;
    theBigOne.mu = (last_gaussian.mu + first_gaussian.mu)/2; 
    MatrixXd pt_0_G0 = gauss2Ellipse(first_gaussian, sigma_rule,1,0,0);
    MatrixXd pt_0_G1 = gauss2Ellipse(last_gaussian, sigma_rule,1,0,0);

    double el_0AngleOffset = atan2(pt_0_G0(1) - first_gaussian.mu(1), pt_0_G0(0) - first_gaussian.mu(0));
    double el_1AngleOffset = atan2(pt_0_G1(1) - last_gaussian.mu(1), pt_0_G1(0) - last_gaussian.mu(0)) ;

    double theta_start_x_G0 = 3*PI/4 + el_0AngleOffset ;
    double theta_end_x_G0 = 5*PI/4 + el_0AngleOffset ;
    double theta_start_x_G1 = PI/4 + el_1AngleOffset ;
    double theta_end_x_G1 = -PI/4 + el_1AngleOffset ;

    //samples ellipse points
    MatrixXd pts_x_G0 = gauss2Ellipse(first_gaussian,sigma_rule,5,theta_start_x_G0,theta_end_x_G0);
    MatrixXd pts_x_G1 = gauss2Ellipse(last_gaussian,sigma_rule,5,theta_start_x_G1,theta_end_x_G1);


    double min_x_G0 = pts_x_G0.col(0).minCoeff();
    double max_x_G1 =  pts_x_G1.col(0).maxCoeff();
  
    double heading = atan2(last_gaussian.mu(1) - first_gaussian.mu(1),last_gaussian.mu(0) - first_gaussian.mu(0));

    double x_majorAxis = fabs(min_x_G0-max_x_G1)*cos(heading);


    double theta_start_y_G0 = sgn(heading)*PI/4 + el_0AngleOffset ;
    double theta_end_y_G0 = sgn(heading)*3*PI/4 + el_0AngleOffset ;
    double theta_start_y_G1 = sgn(heading)* -PI/4 + el_1AngleOffset ;
    double theta_end_y_G1 = sgn(heading)*3*-PI/4 + el_1AngleOffset ;

    //samples ellipse points
    MatrixXd pts_y_G0 = gauss2Ellipse(first_gaussian,sigma_rule,5,theta_start_y_G0,theta_end_y_G0);
    MatrixXd pts_y_G1 = gauss2Ellipse(last_gaussian,sigma_rule,5,theta_start_y_G1,theta_end_y_G1);
    double min_y, max_y;
    //flip sign as per heading
    if (sgn(heading)>0)
    {
        min_y = pts_y_G0.col(1).minCoeff();
        max_y =  pts_y_G1.col(1).maxCoeff();
    }
    else
    {   
        max_y = pts_y_G0.col(1).maxCoeff();
        min_y =  pts_y_G1.col(1).minCoeff();
    };

    double y_majorAxis = fabs(min_y-max_y)*cos(heading);

    double sigma_x = x_majorAxis/(2*sqrt(confidence_scale));
    double sigma_y = y_majorAxis/(2*sqrt(confidence_scale));

    Matrix2d cov ;
    cov << pow(sigma_x,2),0, 0,pow(sigma_y,2);
    Matrix2d R ;
    R<< cos(heading), -sin(heading), sin(heading), cos(heading);
    theBigOne.cov = R*cov*(R.transpose());

    return theBigOne;
};

void intervalGaussian::calcAllIntvlGauss(const vector<Vector3d> mus,const vector<Matrix3d> covs)
{
    intervalGaussian::Gauss firstGauss, lastGauss;
    int jumpSpan = 0;

    for (int i = 0; i< round(maxSpan/ minSpan); i++)
    {
        jumpSpan = (i+1);

        for (int j = 0; j < mus.size()-jumpSpan; j++)
        { 
            firstGauss.mu = mus[j].head(2);
            firstGauss.cov = covs[j].topLeftCorner(2,2);
            lastGauss.mu = mus[j+jumpSpan].head(2);
            lastGauss.cov = covs[j+jumpSpan].topLeftCorner(2,2);
            Store_of_Intvl_Gauss[i].push_back(intervalGaussian::intvlGaussApprox(firstGauss, lastGauss)) ;
        };
    
    };

};

void intervalGaussian::saveTraj(string& filename, intervalGaussian::gaussStrDataType& gaussStr)
{
    //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", ", ", "", "", "", "\n");


    ofstream fileMu(filename + "Mu.csv");
    ofstream fileCov(filename + "Cov.csv");
    // cout<<filename + "Cov.csv"<< endl;

    if (fileMu.is_open() && fileCov.is_open())
    {   
        for(int j; j< gaussStr.size(); j++)
        {
            fileMu << gaussStr[j].mu.format(CSVFormat);
            fileCov<< gaussStr[j].cov.format(CSVFormat);
        };    
    };
    fileMu.close();
    fileCov.close();
};


void intervalGaussian::output_data(string& path)
{   
    for(int i; i< Store_of_Intvl_Gauss.size(); i++)
    {   string filename = path + "/Intvl_" +to_string(i) + "_";
        saveTraj(filename, Store_of_Intvl_Gauss[i]);
    };
};

