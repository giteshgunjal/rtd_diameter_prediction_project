clear; clc; close all;
addpath(genpath(pwd));
% Lena Trang
% 5/10/2023
% Fit an initial covariance for a given time interval given gaussians that
% describe the pdf of a vehicle over an interval


% Starting Heading
% dont need this
% heading0 = pi/3;
% heading1 = -pi/6;
mu_0 = [1 0];
Sigma_0 = [1, 0.0;
           0.0, 1] ...

elpt_1_front = ellipsedata(Sigma_0, mu_0, 10, 3, 0, 2*pi)

% Example gaussian for testing
mu_0 = [-1 5];
Sigma_0 = [0.1, -0.03;
           -0.03, 0.03] ...
         ; % TODO: remove the factor used for visualization

mu_1 = [2 -.5];
Sigma_1 = [0.1, -0.03;
          -0.03, 0.03] ...
         ; % TODO: remove the factor used for visualization


% final gaussian heading
heading = atan2(mu_1(2) - mu_0(2), mu_1(1) - mu_0(1))

%%%%%%%%%%%%%%% Plot for testing %%%%%%%%%%%%%%%%%%%%
plot_pdf(mu_0, Sigma_0);
hold on;
plot_pdf(mu_1, Sigma_1);
c = jet;
colormap(c);
grid off;
shading interp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot for testing

% Find the midpoint of the gaussians 
mu_midpoint = mean([mu_0; mu_1], 1)
% Plot means
scatter(mu_0(1), mu_0(2), "magenta");
scatter(mu_1(1), mu_1(2), "magenta");
scatter(mu_midpoint(1), mu_midpoint(2), "black");

%% Transform the gaussian to an ellipse/find the angle offset
% Find the point at which the zero angle is for the ellipse
elpt_0_angle_0 = ellipsedata(Sigma_0, mu_0, 1, 3, 0, 0)
elpt_1_angle_0 = ellipsedata(Sigma_1, mu_1, 1, 3, 0, 0)

% scatter(elpt_0_angle_0(1), elpt_0_angle_0(2));
% scatter(elpt_1_angle_0(1), elpt_1_angle_0(2));

% Calculate the angle at which the angle 0 is to the world frame x axis
elpt_0_angle_offset = atan2(elpt_0_angle_0(2) - mu_0(2), elpt_0_angle_0(1) - mu_0(1))
elpt_1_angle_offset = atan2(elpt_1_angle_0(2) - mu_1(2), elpt_1_angle_0(1) - mu_1(1))

%% Sample points near back of the first gaussian and the front of the last gaussian
% Define the range for the parts of the ellipse sampled for longitudinal
theta_start_x_back = 3*pi/4 + elpt_0_angle_offset 
theta_end_x_back = 5*pi/4 + elpt_0_angle_offset 
theta_start_x_front = pi/4 + elpt_1_angle_offset 
theta_end_x_front = -pi/4 + elpt_1_angle_offset 

% Sample for points furthest away from each other in the longitudinal axis
elpt_0_back = ellipsedata(Sigma_0, mu_0, 5, 3, theta_start_x_back, theta_end_x_back);
elpt_1_front = ellipsedata(Sigma_1, mu_1, 5, 3, theta_start_x_front, theta_end_x_front);

% Find the minimum/maximum
[~, elpt_0_min_idx] = min(elpt_0_back(:, 1));
elpt_0_x_min = elpt_0_back(elpt_0_min_idx, :)
[~, elpt_1_max_idx] = max(elpt_1_front(:, 1));
elpt_1_x_max = elpt_1_front(elpt_1_max_idx, :)

% Visualize the points for the front and the back
% scatter(elpt_0_back(:, 1), elpt_0_back(:, 2));
% scatter(elpt_1_front(:, 1), elpt_1_front(:, 2));
scatter(elpt_0_x_min(:, 1), elpt_0_x_min(:, 2),'filled', "red");
scatter(elpt_1_x_max(:, 1), elpt_1_x_max(:, 2),'filled', "red");

% Find the distance between the max and min 
x_3_axis = pdist2(elpt_0_x_min(1), elpt_1_x_max(1))*cos(heading)

% Now for y :

%% Sample points near back of the first gaussian and the front of the last gaussian
% Define the range for the parts of the ellipse sampled for longitudinal
theta_start_y_back = sign(heading)*pi/4 + elpt_0_angle_offset 
theta_end_y_back = sign(heading)*3*pi/4 + elpt_0_angle_offset 
theta_start_y_front = sign(heading)*-pi/4 + elpt_1_angle_offset 
theta_end_y_front = sign(heading)*-3*pi/4 + elpt_1_angle_offset 

% Sample for points furthest away from each other in the longitudinal axis
elpt_0_back = ellipsedata(Sigma_0, mu_0, 5, 3, theta_start_y_back, theta_end_y_back);
elpt_1_front = ellipsedata(Sigma_1, mu_1, 5, 3, theta_start_y_front, theta_end_y_front);

% Find the minimum/maximum

% for y need to flip sign as per heading

if sign(heading)>0
    [~, elpt_0_min_idx] = min(elpt_0_back(:, 2));
    elpt_0_y_min = elpt_0_back(elpt_0_min_idx, :)
    [~, elpt_1_max_idx] = max(elpt_1_front(:, 2));
    elpt_1_y_max = elpt_1_front(elpt_1_max_idx, :)

else 
    [~, elpt_0_min_idx] = max(elpt_0_back(:, 2));
    elpt_0_y_min = elpt_0_back(elpt_0_min_idx, :)
    [~, elpt_1_max_idx] = min(elpt_1_front(:, 2));
    elpt_1_y_max = elpt_1_front(elpt_1_max_idx, :)
end

% Visualize the points for the front and the back
% scatter(elpt_0_back(:, 1), elpt_0_back(:, 2));
% scatter(elpt_1_front(:, 1), elpt_1_front(:, 2));
scatter(elpt_0_y_min(:, 1), elpt_0_y_min(:, 2),'filled', "green");
scatter(elpt_1_y_max(:, 1), elpt_1_y_max(:, 2),'filled', "green");

y_3_axis = pdist2(elpt_0_y_min(2), elpt_1_y_max(2))*cos(heading)


scale = 7;
sigma_x = x_3_axis/(2*sqrt(scale))
sigma_y = y_3_axis/(2*sqrt(scale))

cov =  [sigma_x^2 , 0;
        0 ,sigma_y^2];

 R      = [cos(heading) ,-sin(heading); sin(heading), cos(heading)];
rotated_cov = R*cov*R'


epp_final = ellipsedata(rotated_cov, mu_midpoint, 100, 3, 0 , 2*pi);
line(epp_final(:, 1), epp_final(:, 2));
heading



% Calculate new mu
% Calculate new Sigma

function plot_pdf(mu, Sigma)
    num_samples = 1000;
    x = linspace(mu(1)-1, mu(1)+1, num_samples);
    y = linspace(mu(2)-1, mu(2)+1, num_samples);
    [X, Y] = meshgrid(x,y);
    XY = [X(:) Y(:)];
    Z = mvnpdf(XY, mu, Sigma);
    Z = reshape(Z, length(x),length(y));
    
    Z = Z + ones(num_samples, num_samples) * -3;
    surf(X, Y, Z, 'FaceAlpha',0.2);
end


