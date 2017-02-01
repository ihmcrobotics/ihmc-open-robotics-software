package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public class DynamicsConstraintInput extends ICPEqualityConstraintInput
{
   public DynamicsConstraintInput(int maximumNumberOfFreeVariables)
   {
      Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 2);
      beq = new DenseMatrix64F(2, 1);
   }

   public void reshape(int size)
   {
      Aeq.reshape(size, 2);
   }
}
