function ret = createDiscretizationMatrix(A, B, Q)
nStates = size(A, 1);
nInputs = size(B, 2);

ret = [-A',             Q, zeros(nStates, nInputs);
       zeros(size(A)),  A, B;
       zeros(nInputs, 2 * nStates + nInputs)];

end