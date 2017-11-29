package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;

public class SparseDampedNullspaceCalculatorWrapper implements DampedNullspaceCalculator
{
   private final DampedNullspaceCalculator calculator;

   public SparseDampedNullspaceCalculatorWrapper(DampedNullspaceCalculator calculator, int matrixSize)
   {
      this.calculator = calculator;
   }

   @Override
   public void setPseudoInverseAlpha(double alpha)
   {

   }

   @Override
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf)
   {

   }

   @Override
   public void projectOntoNullspace(DenseMatrix64F matrixToProjectOntoNullspace, DenseMatrix64F matrixToComputeNullspaceOf,
                                    DenseMatrix64F projectedMatrixToPack)
   {

   }

   @Override
   public void computeNullspaceProjector(DenseMatrix64F matrixToComputeNullspaceOf, DenseMatrix64F nullspaceProjectorToPack)
   {

   }
}
