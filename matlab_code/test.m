close all; clear;
% x = [0.0240    0.0490    0.0740    0.0990    0.1240    0.1490    0.1740    0.1990    0.2240    0.2490    0.2740;
%    -0.0030   -0.0025   -0.0020   -0.0014   -0.0008   -0.0001    0.0006    0.0014    0.0022    0.0030    0.0040];
% P = [    0.0001    0.0002    0.0003    0.0004    0.0005    0.0006    0.0007    0.0008    0.0009    0.0010    0.0011;
%          0         0         0         0         0         0         0         0         0         0         0;
%     0.0001    0.0002    0.0003    0.0004    0.0005    0.0006    0.0007    0.0008    0.0009    0.0010    0.0011];

x =  [0.3755    0.4005    0.4255    0.4504    0.4754    0.5004    0.5254    0.5503    0.5753    0.6003    0.6252;
    0.0175    0.0186    0.0198    0.0210    0.0222    0.0236    0.0249    0.0264    0.0279    0.0295    0.0311]

P = [0.0001    0.0002    0.0003    0.0004    0.0005    0.0006    0.0007    0.0008    0.0009    0.0010    0.0011;
         0         0         0         0         0         0         0         0         0         0         0;
    0.0001    0.0002    0.0003    0.0004    0.0005    0.0006    0.0007    0.0008    0.0009    0.0010    0.0011]


hold on;
plotUncertainty(x, P);
title("Gaussian Plotted At 0, 5, And 10ms");
xlabel("x");
ylabel("y");
figure(2);
mu1 = x(:, 1)';
mu2 = x(:, 2)';
mu3 = x(:, 3)';
sigma1 = [P(1, 1) P(2, 1);
         P(2, 1) P(3, 1)];
sigma2 = [P(1, 5) P(2, 5);
         P(2, 5) P(3, 5)];
sigma3 = [P(1, 10) P(2, 10);
         P(2, 10) P(3, 10)];

sample_points = 1000;
sigma_multiplier = 100;
X = [mvnrnd(mu1,sigma1*sigma_multiplier,sample_points); mvnrnd(mu2,sigma2*sigma_multiplier,sample_points); mvnrnd(mu3,sigma3*sigma_multiplier,sample_points)];
temp = X;
scatter(X(:, 1), X(:, 2));
g = fitgmdist(X,1);

        mu = g.mu;
        Sigma = g.Sigma;
        num_samples = 100;
        x = linspace(-3 + mu(1), 3 + mu(1), num_samples);
        y = linspace(-3 + mu(2), 3 + mu(2), num_samples);
        [X, Y] = meshgrid(x,y);
        XY = [X(:) Y(:)];
        Z = mvnpdf(XY, mu, Sigma);
        Z = reshape(Z, length(x),length(y));
        
        %Translate to overlap with Zonotope
        Z = Z + ones(num_samples, num_samples) * -0.01;
        surf(X, Y, Z, 'FaceAlpha',0.2);
        c = jet;
        colormap(c);
        grid off;
        shading interp

           title("Fitted Gaussian Sampled From 1000 points");
xlabel("x");
ylabel("y");

hold on ;
% scatter(temp(:, 1), temp(:, 2));





%% Plot Uncertainty
function plotUncertainty(x_predicted, P_predicted)
    for i = 1:3
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
        Z = Z + ones(num_samples, num_samples) * -0.01;
        surf(X, Y, Z, 'FaceAlpha',0.2);
        c = jet;
        colormap(c);
        grid off;
        shading interp
    end
end