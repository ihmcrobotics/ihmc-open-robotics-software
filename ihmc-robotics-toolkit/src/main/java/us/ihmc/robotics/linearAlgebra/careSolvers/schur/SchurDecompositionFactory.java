package us.ihmc.robotics.linearAlgebra.careSolvers.schur;

import org.ejml.data.DMatrixRMaj;

public class SchurDecompositionFactory
{
   public static SchurDecomposition<DMatrixRMaj> qrBased(int size)
   {
      QRBasedSchurDecomposition decomposition = new QRBasedSchurDecomposition(size);
      decomposition.setConvergenceEpsilon(1e-6);
      return decomposition;
   }
}
