package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class ICPQPInputCalculator
{
   public ICPQPInputCalculator(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices)
   {
   }

   public void computeFeedbackTask(ICPQPInput icpQPInput,  DenseMatrix64F feedbackWeight)
   {
      MatrixTools.setMatrixBlock(icpQPInput.quadraticTerm, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
      icpQPInput.linearTerm.zero();
   }
}
