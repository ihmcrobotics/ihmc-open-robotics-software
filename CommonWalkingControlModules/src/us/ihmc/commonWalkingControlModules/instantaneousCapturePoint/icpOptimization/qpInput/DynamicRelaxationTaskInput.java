package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public class DynamicRelaxationTaskInput extends ICPQPInput
{
   public DynamicRelaxationTaskInput()
   {
      quadraticTerm = new DenseMatrix64F(2, 2);
      linearTerm = new DenseMatrix64F(2, 1);
      residualCost = new DenseMatrix64F(1, 1);
   }
}
