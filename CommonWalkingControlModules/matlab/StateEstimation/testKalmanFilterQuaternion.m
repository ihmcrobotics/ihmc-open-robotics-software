function testKalmanFilterQuaternion()
% initial estimated state
state.qHat = [0; 0; 0; 1];
state.omegaHat = [0; 0; 0];
state.bHat = [0; 0; 0];

% noise parameters
qPhi = 0;
qOmega = 1;%1e-2;
qB = 1;
rPhi = 1e-1; %1e-2;
rOmega = 1e-2;

% initial noise covariances
pQ = 1e-1;
pOmega = rOmega;
pOmegaB = 0; %1e-3; % 10 (can be used to make bias estimation faster)
pB = 1e-3; % 10 (can be used to make bias estimation faster)
PQ = pQ * eye(3);
POmegaB = [pOmega * eye(3), -pOmegaB * eye(3);
           -pOmegaB * eye(3), pB * eye(3)];
state.P = blkdiag(PQ, POmegaB);

% initial real state
q0 = normalizeQuaternion([0.6726; 0.0671; -0.0205; 0.7367]);
omega0 = [0.1; -0.1; 0.05];
b0 = [0; -0.05; 0.05];



covariances.QPhi = qPhi * eye(3);
covariances.QB = qB * eye(3);
covariances.QOmega = qOmega * eye(3);
covariances.RPhi = rPhi * eye(3);
covariances.ROmega = rOmega * eye(3);

% simulated data
dt = 5e-3;
tMax = 45;
data = createSimulatedData(dt, tMax, qB, qOmega, rPhi, rOmega, q0, omega0, b0);
% data = createSimulatedConingData(dt, tMax, qB, qOmega, rPhi, rOmega, b0);

n = length(data.t);

% filter
qHat = zeros(4, n);
omegaHat = zeros(3, n);
bHat = zeros(3, n);

tic
for i = 1 : n
    % input
    input.omegad = data.omegad(:, i);

    % measurements
    measurements.zQ = data.zQ(:, i);
    measurements.zOmega = data.zOmega(:, i);

    % filter
    state = kalmanFilterQuaternion(state, input, measurements, covariances, dt);

    % log
    qHat(:, i) = state.qHat;
    omegaHat(:, i) = state.omegaHat;
    bHat(:, i) = state.bHat;
end
filteringTime = toc;
timePerStep = filteringTime / length(data.t);
msPerStep = timePerStep * 1e3;
disp(['filtering time = ' num2str(filteringTime) ' [s], time per step = ' num2str(msPerStep) ' [ms]'])

% data processing
yawPitchRollActual = zeros(3, n);
yawPitchRollMeasured = zeros(3, n);
yawPitchRollEstimated = zeros(3, n);

for i = 1 : n
    qActual = data.q(:, i);
    qMeasured = data.zQ(:, i);
    qEstimated = qHat(:, i);

    RActual = quaternionToRotationMatrix(qActual);
    RMeasured = quaternionToRotationMatrix(qMeasured);
    REstimated = quaternionToRotationMatrix(qEstimated);

    yawPitchRollActual(:, i) = getYawPitchRoll(RActual);
    yawPitchRollMeasured(:, i) = getYawPitchRoll(RMeasured);
    yawPitchRollEstimated(:, i) = getYawPitchRoll(REstimated);
end

% plot
t = data.t;

figure()
for i = 1 : 4
    qActual = data.q(i, :);
    qMeasured = data.zQ(i, :);
    qEstimated = qHat(i, :);
    subplot(2, 2, i), plot(t, qMeasured, 'r', t, qActual, 'g', t, qEstimated, 'b');
    xlabel('t [s]');
    ylabel(['q' num2str(i)]);
    legend({'measured', 'actual', 'estimated'});
end

figure()
angleNames = {'yaw', 'pitch', 'roll'};
for i = 1 : 3
    subplot(3, 1, i), plot(t, yawPitchRollMeasured(i, :), 'r', t, yawPitchRollActual(i, :), 'g', t, yawPitchRollEstimated(i, :), 'b');
    xlabel('t [s]');
    ylabel(angleNames{i});
    legend({'measured', 'actual', 'estimated'});
end

figure();
for i = 1 : 3
    subplot(3, 1, i), plot(t, data.zOmega(i, :), 'r', t, data.omega(i, :), 'g', t, omegaHat(i, :), 'b');
    xlabel('t [s]');
    ylabel(['d\omega' num2str(i) '/dt']);
    legend({'measured', 'actual', 'estimated'});
end

figure();
for i = 1 : 3
    subplot(3, 1, i), plot(t, data.b(i, :), 'g', t, bHat(i, :), 'b');
    xlabel('t [s]');
    ylabel(['b' num2str(i)]);
    legend({'actual', 'estimated'});
end

end

function data = createSimulatedData(dt, tMax, qB, qOmega, rPhi, rOmega, q0, omega0, b0)

randn('seed', 13);

t = 0 : dt : tMax;
n = length(t);

% input
omegad = zeros(3, length(t));
nSines = 3;
amplitudeMin = 0.1;
amplitudeMax = 10;
frequencyMin = 3 * 2 * pi;
frequencyMax = 25 * 2 * pi;
phaseMin = 0;
phaseMax = 2 * pi;
for i = 1 : size(omegad, 1)
    for k = 1 : nSines
        amplitude = amplitudeMin + randn * (amplitudeMax - amplitudeMin);
        frequency = frequencyMin + randn * (frequencyMax - frequencyMin);
        phase = phaseMin + randn * (phaseMax - phaseMin);
        omegad(i, :) = omegad(i, :) + amplitude * sin(frequency * t - phase);
    end
end

% process noise
wOmegad = sqrt(qOmega) * randn(3, n);
wB = sqrt(qB) * randn(3, n);

% state
q = zeros(4, length(t));
omega = zeros(3, length(t));
b = zeros(3, length(t));

q(:, 1) = q0;
omega(:, 1) = omega0;
b(:, 1) = b0;

for i = 1 : length(t) - 1
    qPrev = q(:, i);
    omegaPrev = omega(:, i);
    bPrev = b(:, i);

    qNew = qPrev + 0.5 * quaternionProduct(qPrev, [omegaPrev; 0]) * dt;
    qNew = normalizeQuaternion(qNew);
    omegaNew = omegaPrev + (omegad(:, i) + wOmegad(:, i)) * dt;
    bNew = bPrev + wB(:, i) * dt;

    q(:, i + 1) = qNew;
    omega(:, i + 1) = omegaNew;
    b(:, i + 1) = bNew;
end

% measurement noise
wPhi = sqrt(rPhi) * randn(3, n);
wOmega = sqrt(rOmega) * randn(3, n);

% measurements
zQ = zeros(4, length(t));
zOmega = zeros(3, length(t));
for i = 1 : length(t)
    deltaPhi = wPhi(:, i);
    deltaQ = deltaPhiToDeltaQ(deltaPhi);
    zQ(:, i) = quaternionProduct(deltaQ, q(:, i));
    zOmega(:, i) = omega(:, i) + b(:, i) + wOmega(:, i);
end

data.t = t;
data.omegad = omegad;
data.zQ = zQ;
data.zOmega = zOmega;
data.q = q;
data.omega = omega;
data.b = b;

end

function data = createSimulatedConingData(dt, tMax, qB, qOmega, rPhi, rOmega, b0)

randn('seed', 126);
t = 0 : dt : tMax;
n = length(t);

% process noise
wB = sqrt(qB) * randn(3, n);

% coning trajectory settings
eps = 1; % coning half-angle
frequencyRad = 5;
eX = [1; 0; 0];
eY = [0; 1; 0];
eZ = [0; 0; 1];

% precomputation
sinEps = sin(eps);
sinEps2 = sin(eps / 2);

% q
q = zeros(4, n);
qs = cos(eps / 2);
for i = 1 : n
    ti = t(i);
    qvi = sinEps2 * (sin(frequencyRad * ti) * eX + cos(frequencyRad * ti) * eY);
    qi = packQuaternion(qvi, qs);
    q(:, i) = qi;
end

% omega
omega = zeros(3, n);
for i = 1 : n
    ti = t(i);
    omegai = frequencyRad * (...
        sinEps * cos(frequencyRad * ti) * eX - ...
        sinEps * sin(frequencyRad * ti) * eY + ...
        2 * sinEps2^2 * eZ);
    omega(:, i) = omegai;
end

% omegad
omegad = zeros(3, n);
for i = 1 : n
    ti = t(i);
    omegadi = frequencyRad^2 * (...
        -sinEps * sin(frequencyRad * ti) * eX - ...
        sinEps * cos(frequencyRad * ti) * eY);
    omegad(:, i) = omegadi;
end
wOmegad = sqrt(qOmega) * randn(3, n);
omegad = omegad + wOmegad;

% b
b = zeros(3, length(t));
b(:, 1) = b0;
for i = 1 : length(t) - 1
    bPrev = b(:, i);
    bNew = bPrev + wB(:, i) * dt;
    b(:, i + 1) = bNew;
end

% measurement noise
vPhi = sqrt(rPhi) * randn(3, n);
vOmega = sqrt(rOmega) * randn(3, n);

% measurements
zQ = zeros(4, length(t));
zOmega = zeros(3, length(t));
for i = 1 : length(t)
    deltaPhi = vPhi(:, i);
    deltaQ = deltaPhiToDeltaQ(deltaPhi);
    zQ(:, i) = quaternionProduct(deltaQ, q(:, i));
    zOmega(:, i) = omega(:, i) + b(:, i) + vOmega(:, i);
end

data.t = t;
data.omegad = omegad;
data.zQ = zQ;
data.zOmega = zOmega;
data.q = q;
data.omega = omega;
data.b = b;

end