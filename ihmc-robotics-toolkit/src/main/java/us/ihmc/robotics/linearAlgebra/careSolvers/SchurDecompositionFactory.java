package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;

public class SchurDecompositionFactory
{
   public static SchurDecomposition<DenseMatrix64F> qrBased(int size)
   {
      QRBasedSchurDecomposition decomposition = new QRBasedSchurDecomposition(size);
      decomposition.setConvergenceEpsilon(1e-6);
      return decomposition;
   }
}
