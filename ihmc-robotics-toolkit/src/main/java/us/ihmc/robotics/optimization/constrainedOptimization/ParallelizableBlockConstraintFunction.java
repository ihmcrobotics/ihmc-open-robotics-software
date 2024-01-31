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
 *
 * This version of the block constraint function allows for parallelization of global resources.
 * Resource management is done globally, which is why only the problem index is passed
 * and not the problem itself. The isolated problem also should not know about global resources.
 */
public interface ParallelizableBlockConstraintFunction
{
   double calculate(DMatrixD1[] blocks, int problemIndex);
}
