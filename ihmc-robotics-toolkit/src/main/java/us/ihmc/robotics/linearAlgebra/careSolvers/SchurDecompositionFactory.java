package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;

public class SchurDecompositionFactory
{
   public static SchurDecomposition<DenseMatrix64F> qrBased(int size)
   {
      return new QRBasedSchurDecomposition(size);
   }
}
