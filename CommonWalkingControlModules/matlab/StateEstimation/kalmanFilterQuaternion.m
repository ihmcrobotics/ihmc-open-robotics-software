function state = kalmanFilterQuaternion(state, input, measurements, covariances, dt)

% unpack
qHat = state.qHat;
omegaHat = state.omegaHat;
bHat = state.bHat;
P = state.P;

omegad = input.omegad;
QPhi = covariances.QPhi;
QOmega = covariances.QOmega;
QB = covariances.QB;

zQ = measurements.zQ;
zOmega = measurements.zOmega;
RPhi = covariances.RPhi;
ROmega = covariances.ROmega;

% create system
[A, B] = createContinuousTimeSystem(omegaHat);
QContinuous = blkdiag(QPhi, QOmega, QB);
RContinuous = blkdiag(RPhi, ROmega);
% BW = eye(9);
% [F, G, Q, R] = discretize(A, B, BW, QContinuous, RContinuous, dt);
[F, G, Q, R] = discretize2(A, B, QContinuous, RContinuous, dt);
H = [-eye(3), zeros(3, 6);
     zeros(3, 3), eye(3), eye(3)];

% a priori state
% omegaHat is expressed in body frame, so to obtain the new rotation from
% body to world, we need to postmultiply it:
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% q^W_NB = q^W_OB * q^OB_NB
% q^W_OB = qHat
% q^OB_NB = deltaPhiToDeltaQ(omegaHat * dt)
qHat = quaternionProduct(qHat, deltaPhiToDeltaQ(omegaHat * dt));% + omegad * dt^2 / 2)); 
omegaHat = omegaHat + omegad * dt;

% a priori covariance
P = F * P * F' + Q;

% measurement residual
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% * let M be measured frame
% q^W_OB = qHat
% in the end, we want deltaQ = deltaPhiToDeltaQ(deltaPhi) = q^OB_NB
% with deltaPhi = KPhi * deltaQToDeltaPhi(q^OB_M)
% hence, the error term yQ needs to be a correction in body frame: 
% yQ = q^OB_M
% q^OB_M = q^OB_W * q^W_M = (q^W_OB)^-1 * zQ
yQ = quaternionProduct(quaternionInverse(zQ), qHat);
yPhi = deltaQToDeltaPhi(yQ);
yOmega = zOmega - (omegaHat + bHat);
y = [yPhi; yOmega];

% Kalman gain matrix
S = H * P * H' + R;
K = P * H' * inv(S);

% a posteriori state
deltaX = K * y;
deltaPhi = deltaX(1 : 3);
deltaQ = deltaPhiToDeltaQ(deltaPhi);
deltaOmega = deltaX(4 : 6);
deltaB = deltaX(7 : 9);

% deltaQ is expressed in body frame, so to obtain the new rotation from
% body to world, we need to postmultiply it:
% * let W be world frame,
% * let OB be old body frame
% * let NB be new body frame
% q^W_NB = q^W_OB * q^OB_NB
% with
% q^W_OB = qHat
% q^OB_NB = deltaQ
% so: qHat <- qHat * deltaQ
qHat = quaternionProduct(qHat, deltaQ);
qHat = normalizeQuaternion(qHat);
omegaHat = omegaHat + deltaOmega;
bHat = bHat + deltaB;

% a posteriori covariance
P = (eye(size(P, 1)) - K * H) * P;

% pack
state.qHat = qHat;
state.omegaHat = omegaHat;
state.bHat = bHat;
state.P = P;

end



function [A, B] = createContinuousTimeSystem(omegaHat)
% no cross product
% A = [zeros(3, 3), eye(3), zeros(3, 3);
%      zeros(3, 9);
%      zeros(3, 9)];

% cross product coefficient 1
% A = [-tilde(omegaHat), eye(3), zeros(3, 3);
%      zeros(3, 9);
%      zeros(3, 9)];

% cross product coefficient 1/2
A = [-1/2 * tilde(omegaHat), eye(3), zeros(3, 3);
     zeros(3, 9);
     zeros(3, 9)];

B = [zeros(3, 3);
     eye(3);
     zeros(3, 3)];
% B = zeros(9, 0);
end


