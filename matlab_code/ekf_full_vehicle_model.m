classdef ekf_full_vehicle_model < handle   
    properties
        A                   % system matrix Jacobian
        B;                  % input matrix Jacobian
        H;                  % measurement model Jacobian
        S;                  % innovation covariance
        v;                  % innovation
        f;                  % process model
        h;                  % measurement model
        x;                  % state vector [x; y; h]
        z_hat;              % predicted measurement
        Sigma;              % state covariance
        x_pred;             % predicted state
        Sigma_pred;         % predicted state covariance
        Q;                  % input noise covariance
        R;                  % measurement noise covariance
        K;                  % Kalman (filter) gain
        xdot;               % Velocities [Longitudinal; Lateral; Angular]
        prev_state;         % Previous State [x; y; h]
        dt                  % delta time
        forward_predict_x;  % vector to store forward_predicted_states
        forward_predict_P;  % vector to store convariances 
        
    end
    
    methods
        function obj = ekf_full_vehicle_model(system, init)
            % ekf Construct an instance of this class
            %
            %   Inputs:
            %       system          - system and noise models
            %       init            - initial state mean and covariance
            
            obj.A = system.A;
            obj.B = system.B;
            obj.f = system.f;
            obj.H = system.H;
            obj.Q = system.Q;
            obj.R = system.R;
            obj.h = system.h;
            obj.x = init.x;
            obj.Sigma = init.Sigma;
            obj.xdot = init.xdot;
            obj.prev_state = init.x;
            obj.dt = system.dt;
        end
        
        function prediction(obj)
            % EKF propagation (prediction) step
            obj.x_pred = obj.f(obj.x, obj.xdot, obj.dt);
            obj.Sigma_pred = obj.A * obj.Sigma * obj.A' + obj.Q;
            obj.z_hat = obj.h(obj.x_pred);
            obj.x = obj.x_pred;
            obj.Sigma = obj.Sigma_pred;
        end

        function forward_predict(obj)
            % EKF propagation (prediction) step
            x_prev = obj.x;
            sigma_prev = obj.Sigma;
            obj.forward_predict_x(:, 1) = x_prev;
            obj.forward_predict_P = cat(3, obj.forward_predict_P, sigma_prev);

            for i = 2:11
                x_new = obj.f(x_prev, obj.xdot, obj.dt);
                sigma_prev = obj.A * sigma_prev * obj.A' + obj.Q;
                
                obj.forward_predict_x(:, i) = x_new;
                obj.forward_predict_P = cat(3, obj.forward_predict_P, sigma_prev);
                x_prev = x_new;
            end
        end
        
        function correction(obj, z)
            % EKF correction step
            %
            %   Inputs:
            %       z          - measurement
            
            % evaluate measurement Jacobian at current operating point
            H = obj.H;
            
            % compute innovation statistics
            obj.v = z - obj.z_hat;
            obj.S = H * obj.Sigma_pred * H' + obj.R;
            
            % filter gain
            obj.K = obj.Sigma_pred * H' * (obj.S \ eye(size(obj.S)));
            
            % correct the predicted state statistics
            obj.x = obj.x_pred + obj.K * obj.v;
            I = eye(length(obj.x));
            obj.Sigma = ...
                (I - obj.K * H) * obj.Sigma_pred * (I - obj.K * H)' ...
                    + obj.K * obj.R * obj.K'; % Joseph update form
            obj.forward_predict();  % Run forward predict after SLAM update
        end
    end
end