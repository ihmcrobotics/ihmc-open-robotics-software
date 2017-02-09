package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public abstract class ICPEqualityConstraintInput
{
   public DenseMatrix64F Aeq;
   public DenseMatrix64F beq;

   public void reset()
   {
      Aeq.zero();
      beq.zero();
   }
}
