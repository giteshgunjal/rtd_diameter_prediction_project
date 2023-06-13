#include "utility.h"
#include "kalman_filter.h"
#include "interval_gaussian_helper.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
using namespace std::chrono;


class velo_GPR :public kernals 
{
public:

    ros::NodeHandle nh;
    ros::Subscriber subOdom;
    string odomTopic;
    vector<double> velo_info;
    queue<vector<double> > velo_buffer;

    static constexpr int buffer_size = 100;
    static constexpr int layers = 4;
    static constexpr double pred_horizon = 3;
    static constexpr double time_step = 1e-2;

    using subSampledDataType = Matrix<double,int (buffer_size/layers),1>;
    subSampledDataType sublinearVelocityData;
    subSampledDataType subangularVelocityData;
    subSampledDataType subtimeData;

    using dataBufferType = Matrix<double,buffer_size,1> ;
    dataBufferType linearVelocityBuffer;
    dataBufferType angularVelocityBuffer;
    dataBufferType timeBuffer;

    using predVecDataType = Matrix<double,buffer_size+ int (pred_horizon/time_step) ,1> ;
    predVecDataType timeData;
    predVecDataType linearVeloMean;
    predVecDataType angularVeloMean;

    using predVeloDataType = Matrix<double,2, int (pred_horizon/time_step)> ;
    predVeloDataType predVelocity;

    using predCovDataType =  Matrix<double,buffer_size+ int (pred_horizon/time_step) ,buffer_size+ int (pred_horizon/time_step) > ;
    predCovDataType linearVeloCov;
    predCovDataType angularVeloCov;
     
    vector<Vector3d> meanVectorStr;

    vector<Matrix3d> covMatrixStr;

    bool bufferFull; 
    bool doneFirst;

    // initail state , uncertainty and process noise; 
    VectorXd X_in;
    MatrixXd P_in;
    MatrixXd Q_in;
    
    velo_GPR(): odomTopic{"/vesc/odom"},bufferFull{false}, doneFirst{false} 
    {   meanVectorStr.resize(int (pred_horizon/time_step));
        covMatrixStr.resize(int (pred_horizon/time_step));
        X_in = Vector3d(0.0,0.0,0.0);
        P_in = Matrix3d(Vector3d(pow(10,-3),pow(10,-3),pow(10,-3)).asDiagonal());
        Q_in = Matrix3d(Vector3d(pow(10,-3),pow(10,-3),pow(10,-3)).asDiagonal());
    }


    void odom_handler(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        velo_info.assign({odom_msg->header.stamp.sec + 1e-9* odom_msg->header.stamp.nsec,odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z});
        velo_buffer.push(velo_info);
        // if (doneFirst){ros::shutdown();};
        // cout<< velo_buffer.size();
        if (velo_buffer.size()> buffer_size)
        {   //cout<<"Array full"<<endl;
            bufferFull = true;
            velo_buffer.pop();
            // auto start = high_resolution_clock::now();
            traj_pred();



            prop_uncertainty();
            cout<< "hey1"<<endl;
            intervalGaussian invG;
            
            string file_path = "ekf_test/";
            saveTraj(file_path,meanVectorStr );
            
            invG.calcAllIntvlGauss(meanVectorStr, covMatrixStr);
            cout<< "hey2"<<endl;
            file_path = "output";
            invG.output_data(file_path);
            cout<< "hey3"<<endl;
            return;

            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(stop - start);
            // // cout << duration.count() << endl;  
        };
    };

    void prop_uncertainty()
    {   
        KalmanFilter ekf;
        ekf.Init(X_in, P_in, Q_in);
        predVelocity.row(0) = linearVeloMean.tail(int (pred_horizon/time_step));
        predVelocity.row(1) = angularVeloMean.tail(int (pred_horizon/time_step));
        string file_path = "pred_test.csv";
        saveData(file_path, predVelocity);
        double dt = time_step;
        for (int i =0 ; i< int (pred_horizon/time_step); i++)
        {
            meanVectorStr[i] = ekf.x_;  
            covMatrixStr[i] = ekf.P_;
            ekf.Predict(predVelocity.col(i), dt);
            
        };
    }

// no need to scale time we are going to do a recursive bayesian update. gpr output will be cov 

    MatrixXd build_cov(const Ref<const VectorXd>& x, const Ref<const VectorXd>& xp)
    {
        MatrixXd out;
        out.resize(x.size(),xp.size());

        for(int i = 0; i <xp.size(); i++ )
        {
            out.col(i) = sq_exp(x, xp(i));
        };
        return out;
    };

    void gpr(const subSampledDataType& xtrain, const subSampledDataType& ytrain, const predVecDataType& xpred,const subSampledDataType& pred_data,  predVecDataType &mean)
    {  
        MatrixXd  cov = build_cov(xtrain, xtrain) + MatrixXd::Identity(xtrain.size(),xtrain.size()) * pow(10,log_noise_learn);
        MatrixXd invcov = cov.inverse();
        MatrixXd vec_pred = build_cov(xpred,xtrain);
        mean = mean + vec_pred * (invcov*(ytrain - pred_data));

        // const_cast<MatrixXd &>(pred_cov) = build_cov(xpred, xpred)  - vec_pred*invcov*vec_pred.transpose();

        return;
    };

 

    void traj_pred()
    {
        // if (velo_buffer.size()!= buffer_size)
        // {
        //     // buffer not full
        //     return;
        // };

        double start_time = velo_buffer.front()[0];
        vector<double> currVeloInfo;
        int count =1;
        for (int i=0; i < buffer_size; i++)
        {   
            currVeloInfo = velo_buffer.front();
            velo_buffer.pop();
            timeBuffer[i] = currVeloInfo[0]- start_time;
            linearVelocityBuffer[i] = currVeloInfo[1];
            angularVelocityBuffer[i] = currVeloInfo[2];
        };
        // get time horizon:
        timeData.head(buffer_size) = timeBuffer;
        timeData.tail(timeData.size()- buffer_size +1) = VectorXd::LinSpaced(timeData.size()- buffer_size +1,0,pred_horizon).array() +  double(timeBuffer[timeBuffer.size()-1]);

        for (int i =0; i < layers ; i++)
        {   
            subtimeData = timeBuffer.segment(i* int (buffer_size/layers) ,int (buffer_size/layers) );
            sublinearVelocityData = linearVelocityBuffer.segment(i* int (buffer_size/layers) ,int (buffer_size/layers) );
            subangularVelocityData = angularVelocityBuffer.segment(i* int (buffer_size/layers) , int (buffer_size/layers) );
            
            if (i ==0)
            {   
                linearVeloMean.setConstant(linearVelocityBuffer.mean());
                angularVeloMean.setConstant(angularVelocityBuffer.mean());

                linearVeloCov= build_cov(timeData, timeData);
                angularVeloCov = linearVeloCov;
 
            };
            
            gpr(subtimeData,sublinearVelocityData,timeData,linearVeloMean.segment(i* int (buffer_size/layers) ,int (buffer_size/layers) ), linearVeloMean);
            gpr(subtimeData,subangularVelocityData,timeData,angularVeloMean.segment(i* int (buffer_size/layers) ,int (buffer_size/layers) ), angularVeloMean);

            
        };
        doneFirst = true;  
        cout<< "GPR done!"<<endl;
    };

};



int main(int argc, char** argv)
{
    

    // // diag = Matrix2d (v.asDiagonal());
    // Vector3d l;
    // cout<< v.asDiagonal()<< endl;
    ros::init(argc, argv, "robot");
    velo_GPR Velo_pred;

    Velo_pred.subOdom = Velo_pred.nh.subscribe<nav_msgs::Odometry>  (Velo_pred.odomTopic,2000, &velo_GPR::odom_handler, &Velo_pred,ros::TransportHints().tcpNoDelay());
    // cout<<Velo_pred.doneFirst<<endl;
    // if (Velo_pred.bufferFull){
    //     cout<<"start gpr"<<endl;
    //     Velo_pred.traj_pred();
    // };

    ros::spin();

    if (Velo_pred.doneFirst)
    {return 0;};
    
}