% Lena Trang
% Adapted from ekf_dynamic_target.m by Vishrut Kaushik
% 4/5/2023

clc; clear; close all

% vel derived from slam
% u, v, r desired? NO because obstacle rover
% vehicle states x, y, h, u, v, r, w, delta_cmd

% goals --> have uncertainty progrogation for the future 4.5s

% We assume that the state we are in has some uncertainty
% Thus, our real state x_actual = x_pred + x_error

% Then, we can predict where we will be next
% So, x_pred_next = x_pred + xdot
% We can extend this to say that for every possible predicted place, 
% the gaussian extends to be more spread out

% then, using the slam prediction, we can re-evaluate the state

% HOWEVER, we can only do this continuously. When we submit the
% progrogation to the planner, it needs to be for 4.5 seconds

% Note that gitesh will provide the predicted velocities for the future 4.5
% seconds. Thus, our input will vary.

% So, the goal is to take in gitesh's vector of velocities for every 0.1s,
% and predict the pdf based on that

% we can reevaluate it every 0.1s to gain a better state estimate, but we
% cannot improve the pdf submitted to the planner as that would require
% knowledge of the future



% Vector for current state
% x = [x_; y_; h_; u_; v_; r_];
x = [0; 0; 0; 1; 0; 0];

% import vector of velocities for the future n seconds
n = 4.5;
dt = 0.1;
state_pred = NaN([6, size(1:dt:n, 2)]);

% Read in the vector of velocities

% pred_velocities_u = 
pred_velocities_v = zeros(1, size(1:dt:n, 2));
% pred_velocities_r = 

% state_pred(4, :) = pred_velocities_u;
state_pred(5, :) = pred_velocities_v;
% state_pred(6, :) = pred_velocities_r;

% Function for x_dot
x_dot = @(u, v, h, r) [u * cos(h) - v * sin(h); u * sin(h) + v * cos(h); r];
% Function for predicted state
x_pred = @(x, dt) x(1:3) + x_dot(x(4), x(5), x(3), x(6)) * dt;












%% Build the system
sys = [];
sys.A = eye(3);
sys.B = [];
sys.f = x_pred; % NOTE, this is changed to contain the heading as well

% sys.H = @() [x(1)/(x(1)^2 + x(2)^2)^(1/2), x(2)/(x(1)^2 + x(2)^2)^(1/2);
%               x(2)/(x(1)^2 + x(2)^2), -x(1)/(x(1)^2 + x(2)^2)];
sys.H = eye(3);
sys.h = @(x)  [x(1); x(2); x(3)];
sys.Q = 1e-4 * eye(3);
% sys.R = diag([0.01^2; 0.01^2; 0.01^2]);
sys.R = 1e-4 * eye(3);
sys.dt = dt;

%% Initilize the state using the current measurement
init = [];
init.x = zeros(3,1);
init.Sigma = 1 * eye(3);
init.linear_velocity = x(4);
init.lateral_velocity = x(5);
init.heading = x(3);











%% Run EKF
filter = ekf(sys, init);
x = init.x;     % state
P(:, 1) = [init.Sigma(1, 1), init.Sigma(1, 2), init.Sigma(2, 2)];
% main loop; iterate over the measurements
count = 0;
for i = 1:size(z,2)
    filter.prediction();
    if z(:, i) == -1 
        count = count + 1;
        x(:,i) = filter.x;
        P(:,i) = [filter.Sigma(1, 1), filter.Sigma(1, 2), filter.Sigma(2, 2)];
        continue
    end

    filter.correction(z(:,i));
    x(:,i) = filter.x;
    P(:,i) = [filter.Sigma(1, 1), filter.Sigma(1, 2), filter.Sigma(2, 2)];
    plotUncertainty(filter.forward_predict_x, filter.forward_predict_P);
end

axis equal
scatter(x(1, :), x(2, :), 'blue', 'DisplayName','Filter Tracking');

%% Plot Uncertainty
function plotUncertainty(x_predicted, P_predicted)
    for i = 1:size(x_predicted, 2)
        mu = [x_predicted(1, i), x_predicted(2, i)];
        Sigma = 1e2*[P_predicted(1, i) P_predicted(2, i); ...
                 P_predicted(2, i) P_predicted(3, i)];
        num_samples = 100;
        x = linspace(-3, 3, num_samples);
        y = linspace(-3, 3, num_samples);
        [X, Y] = meshgrid(x,y);
        XY = [X(:) Y(:)];
        Z = mvnpdf(XY, mu, Sigma);
        Z = reshape(Z, length(x),length(y));
        
        %Translate to overlap with Zonotope
        X = X + ones(num_samples, num_samples) * mu(1);
        Y = Y + ones(num_samples, num_samples) * mu(2);
        Z = Z + ones(num_samples, num_samples) * -0.01;
        surf(X, Y, Z, 'FaceAlpha',0.2);
        c = jet;
        colormap(c);
        grid off;
        shading interp
    end
end












