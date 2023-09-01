package us.ihmc.robotics.optimization.constrainedOptimization;

import org.ejml.data.DMatrixD1;

/**
 * For the optimization problem:
 *    minimize f1(x1) + f2(x2) + ...
 *    st:
 *       G1[](x1) == 0, G2[](x2) == 0, ...
 *       H1[](x1) >= 0, H2[](x2) >= 0, ...
 *
 *       J[](x1, x2,...) == 0
 *       K[](x1, x2,...) >= 0,
 *
 * a block is one vector xi associated with the individual problem, min fi(xi)
 * This interface is used to evaluate J[] and K[], which operate over all blocks.
 */
public interface BlockConstraintFunction
{
   double calculate(DMatrixD1[] blocks);
}
