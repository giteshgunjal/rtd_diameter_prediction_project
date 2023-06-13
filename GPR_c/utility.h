#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <queue>
#include <vector>
#include <string>
#include <iostream>
#include<fstream>


using namespace std;
using namespace Eigen;

// define Kernals:

class kernals
{
public:

    // kernal params :
    float sq_exp_tau = 1.1;
    float sq_exp_log_width = .6;
    float period_tau  = 1;
    float period_log_width = 0.5;
    float period_p = 0.5;
    float poly_c = 1;
    float poly_d = 3;
    float log_noise_learn= -3.;
  
    VectorXd sq_exp(const Ref<const VectorXd>& x, const double& xp)
    {   
        return (pow(sq_exp_tau,2)* (-0.5 * (x.array()-xp).square()/ pow(pow(10,sq_exp_log_width),2)).exp());
    };

    VectorXd periodic(const Ref<const VectorXd>& x, const double& xp)
    {
        return (pow(period_tau,2)* (-2 * (M_PI*  (x.array()-xp).abs() /period_p).sin().pow(2)    / pow(pow(10,period_log_width),2)).exp());
    };

    VectorXd poly(const Ref<const VectorXd>& x, const double& xp)
    {
        return ((x*xp).array() +poly_c).pow(poly_d) ;
    };

    void saveData(string fileName, const Ref<const MatrixXd>&  matrix)
    {
        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    
        ofstream file(fileName);
        if (file.is_open())
        {
            file << matrix.format(CSVFormat);
            file.close();
        }
    };

    void saveTraj(string& filename, vector<Matrix<double, 3, 1> >& meanStr)
    {
        //https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
        const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", ", ", "", "", "", "\n");


        ofstream fileMu(filename + "Mu.csv");
        // ofstream fileCov(filename + "Cov.csv");
        // cout<<filename + "Cov.csv"<< endl;
        // cout<<gaussStr.size()<< endl;
        if (fileMu.is_open())
        {   
            for(int j; j< meanStr.size(); j++)
            {
                fileMu << meanStr[j].format(CSVFormat);
                // fileCov<< gaussStr[j].cov.format(CSVFormat);
            };    
        };
        fileMu.close();
        // fileCov.close();

    }

    // VectorXd sq_exp(VectorXd x, double xp)
    // {
    //     return ((x*xp).array() +poly_c).pow(poly_d) ;
    // };

};