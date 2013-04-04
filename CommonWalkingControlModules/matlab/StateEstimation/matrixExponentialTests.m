function matrixExponentialTests()

close all;

nBiases = 2;
AR = rand(6, 6);
AQ1 = rand(6, 6);
AQ2 = rand(6, 6);
AB = rand(nBiases * 3, nBiases * 3);
A1 = blkdiag(AR, AQ1, AB);
A2 = blkdiag(AR, AQ2, AB);
nStates = size(A1, 1);

BR = rand(6, 3);
BQ = rand(6, 3);
BB = zeros(3 * nBiases, 6);
B = [BR, zeros(6, 3);
     zeros(6, 3), BQ;
     BB];
nInputs = size(B, 2);

% B = rand(nStates, 6);

QR = rand(6, 6);
QQ = rand(6, 6);
QB = rand(nBiases * 3, nBiases * 3);
Q = blkdiag(QR, QQ, QB);

M1 = createDiscretizationMatrix(A1, B, Q);
M2 = createDiscretizationMatrix(A2, B, Q);

expM1 = expm(M1);
expM2 = expm(M2);

diff = expM1 - expM2;

figure();
hold on;
spy(diff)

discretizationMatrixSize = size(M1, 1);
plot([0, discretizationMatrixSize], [nStates, nStates], 'k');
plot([0, discretizationMatrixSize], [2 * nStates, 2 * nStates], 'k');
plot([nStates, nStates], [0, discretizationMatrixSize], 'k');
plot([2 * nStates, 2 * nStates], [0, discretizationMatrixSize], 'k');

F3Q1 = expM1(nStates + 7 : nStates + 12, nStates + 7 : nStates + 12);
G2Q1 = expM1(7 : 12, nStates + 7 : nStates + 12);
G3Q1 = expM1(nStates + 7 : nStates + 12, 2 * nStates + 4 : 2 * nStates + 6);

MSmall = createDiscretizationMatrix(AQ1, BQ, QQ);
expMSmall = expm(MSmall);

F3Q1Small = expMSmall(7 : 12, 7 : 12);
G2Q1Small = expMSmall(1 : 6, 7 : 12);
G3Q1Small = expMSmall(7 : 12, 13 : 15);

F3Q1Diff = F3Q1 - F3Q1Small;
G2Q1Diff = G2Q1 - G2Q1Small;
G3Q1Diff = G3Q1 - G3Q1Small;

disp(norm(F3Q1Diff, inf));
disp(norm(G2Q1Diff, inf));
disp(norm(G3Q1Diff, inf));

end