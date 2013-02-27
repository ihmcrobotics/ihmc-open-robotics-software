% state: [xn; xdn; xddn; xddn-1]
% input: f


randn('seed', 123)
dt = 5e-3;
m = 5;

%% model
A = [1, dt, dt^2 / 2, 0;
     0, 1,  dt,       0;
     0, 0,  0,        0;
     0, 0,  1,        0];

B = [0;
     0;
     1 / m;
     0];

C = [1, 0, 0, 0;
     0, 0, 0, 1 / m];

D = [0;
     0];

stateNames = {'x' 'xd' 'xdd' 'xddPrev'};
sys = ss(A, B, C, D, dt, 'StateName', stateNames, 'InputName', 'f');

fProcessCovariance = 1e-1;
rCovariance = 1e-5;
fMeasCovariance = 1e-5;
Q = diag([0, 0, fProcessCovariance, 0]);
R = [rCovariance, 0;
     0, fMeasCovariance];

%% simulated data
tMax = 5;
t = 0 : dt : tMax;

rdd = zeros(1, length(t));
nSines = 3;
amplitudeMin = 0.1;
amplitudeMax = 10;
omegaMin = 3 * 2 * pi;
omegaMax = 25 * 2 * pi;
phaseMin = 0;
phaseMax = 2 * pi;
for i = 1 : nSines
    amplitude = amplitudeMin + randn * (amplitudeMax - amplitudeMin);
    omega = omegaMin + randn * (omegaMax - omegaMin);
    phase = phaseMin + randn * (phaseMax - phaseMin);
    rdd = rdd + amplitude * sin(omega * t - phase);
end

f = rdd / m;
%fPrev = [0, f(1 : length(f) - 1)];
fPrev = zeros(size(f, 1), size(f, 2));

randn('seed', 0)
n = length(t);

w = sqrt(fProcessCovariance) * randn(1, n);  % w = process noise
v = (sqrt(R) * randn(size(C, 1), n)); % v = measurement noise

% yPerfect = [r; fPrev];
% yNoisy = yPerfect + v;
u = f + w;
x0 = zeros(size(A, 1), 1);
[yPerfect, t, x] = lsim(sys, u, t, x0);
yPerfect = yPerfect';
x = x';

yNoisy = yPerfect + v;

%% Kalman filtering
s.P = diag([100, 100, 100, 100]); % initial error covariance
s.x = zeros(size(A, 1), 1);     % Initial condition on the state
s.A = A;
s.B = B;
s.H = C;
s.Q = Q;
s.R = R;

xHat = zeros(size(A, 1), length(t));
yHat = zeros(size(C, 1), length(t));
PHat = cell(length(t));

for i=1:length(t)
    s.u = f(i);
    s.z = yNoisy(:, i);
    s = kalmanf(s);
    xHat(:, i) = s.x;
    yHat(:, i) = C * s.x;
    PHat{i} = s.P;
end

%% Plots
% close all
timeResponseLabels = {'position [m]', 'force [N]'};
errorLabels = {'position error [m]', 'force error [N]'};

for i = 1 : size(yPerfect, 1)
    figure();
    subplot(211), plot(t, yPerfect(i, :),'--', t, yHat(i, :),'-')
    title('Time response');
    xlabel('t [sec]'), ylabel(timeResponseLabels{i})
    legend('measured', 'estimated');
    axis tight;

    subplot(212), plot(t, yPerfect(i, :) - yNoisy(i, :),'-.',t, yPerfect(i, :) - yHat(i, :),'-')
    xlabel('t [sec]'), ylabel(errorLabels{i});
    legendHandle = legend('measured', 'estimated');
    axis tight;
end

figure();
nStates = size(xHat, 1);
nRows = 2;
nColumns = ceil(nStates / nRows);
for i = 1 : nStates
    subplot(nRows, nColumns, i), plot(t, x(i, :), t, xHat(i, :))
    xlabel('t [sec]');
    ylabel(stateNames{i});
    legend('actual', 'estimated');
    axis tight;
end
