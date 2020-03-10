package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

import org.ejml.data.DenseMatrix64F;

/**
 * If an nxn matrix K, has a Jordan canconical form
 * K = MJMinv = M(D + N) Minv
 *
 * Then the matrix sign function of K is defined as
 * sign(K) = MSMinv = W
 * where S is a diagonal matrix whose
 * entries are given by
 * 1 if Re(d) > 0 and -1 if Re(d) < 0
 *
 * If one of the eigenvalues of K lies on the imaginary axis,
 * sign(K) is undefined.
 *
 */
public interface SignFunction
{
   boolean compute(DenseMatrix64F K);

   DenseMatrix64F getW(DenseMatrix64F WToPack);
}
