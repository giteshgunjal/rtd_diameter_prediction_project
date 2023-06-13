clc; clear; close all;

%% Generate Ground Truth Data
% Constant Velocity Vehicle Model
vehicle_model = @(u, v, h, r) [u * cos(h) - v * sin(h), u * sin(h) + v * cos(h), r];
observations = 500;
t = linspace(0, 10, observations);
x = linspace(0, 10, observations); 
y = x;
% y = 3*sin(x);
% y = zeros(size(x));
h = atan2((y(2:end) - y(1:end-1)), (x(2:end) - x(1:end-1)));
h = [0, h];

plot(x, y);
gt = zeros(length(h)+1, 3);
u = 5;
v = 0.01;
r = (h(2:end) - h(1:end-1)) ./ (t(2:end) - t(1:end-1));
gt(1, :) = [0, 0, 0];
for i = 2:length(h)
    gt(i, :) = gt(i-1, :) + vehicle_model(u, v, gt(i-1, 3), r(i-1)) * (t(i) - t(i-1));
end
hold on
zlim([0, 10]);
scatter(gt(:, 1), gt(:, 2), 'red', 'DisplayName','Ground Truth');

%% Generate Sensor Measurements
R = diag([0.02^2; 0.01^2; 0.01^2]);
L = chol(R, 'lower');
z = [];
for i = 1:length(gt)
   noise = L * randn(3,1); 
   if mod(i, 5) == 1
%        z(:,i) = [sqrt(gt(i, 1)^2 + gt(i, 2)^2); atan2(gt(i, 1), gt(i, 2))] + noise;
       z(:,i) = [gt(i, 1); gt(i, 2); gt(i, 3)] + noise;       % SLAM Measurements
   else
       z(:,i) = [-1; -1; -1];
   end
end

%% Build the system
sys = [];
sys.A = eye(2);
sys.B = [];
sys.f = @(x, u, v, h, dt) [x(1) + (u * cos(h) - v * sin(h))*dt ; x(2) + (u * sin(h) + v * cos(h))*dt];
% sys.H = @() [x(1)/(x(1)^2 + x(2)^2)^(1/2), x(2)/(x(1)^2 + x(2)^2)^(1/2);
%               x(2)/(x(1)^2 + x(2)^2), -x(1)/(x(1)^2 + x(2)^2)];
sys.H = eye(2);
sys.h = @(x)  [x(1); x(2)];
sys.Q = 1e-4 * eye(2);
% sys.R = diag([0.01^2; 0.01^2]);
sys.R = 1e-4 * eye(2);
sys.dt = t(2) - t(1);

%% Initilize the state using the first measurement
init = [];
init.x = zeros(2,1);
init.Sigma = 1 * eye(2);
init.linear_velocity = u;
init.lateral_velocity = v;
init.heading = h(1);

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
        x = linspace(-3 + mu(1), 3 + mu(1), num_samples);
        y = linspace(-3 + mu(2), 3 + mu(2), num_samples);
        [X, Y] = meshgrid(x,y);
        XY = [X(:) Y(:)];
        Z = mvnpdf(XY, mu, Sigma);
        Z = reshape(Z, length(x),length(y));
        
        %Translate to overlap with Zonotope
%         X = X + ones(num_samples, num_samples) * mu(1);
%         Y = Y + ones(num_samples, num_samples) * mu(2);
        Z = Z + ones(num_samples, num_samples) * -0.01;
        surf(X, Y, Z, 'FaceAlpha',0.2);
        c = jet;
        colormap(c);
        grid off;
        shading interp
    end
end

