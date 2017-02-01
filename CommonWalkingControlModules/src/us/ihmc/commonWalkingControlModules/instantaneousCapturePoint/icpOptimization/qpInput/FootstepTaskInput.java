package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;

public class FootstepTaskInput extends ICPQPInput
{
   public FootstepTaskInput(int maximumNumberOfFootstepsToConsider)
   {
      quadraticTerm = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      linearTerm = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      residualCost = new DenseMatrix64F(1, 1);
   }

   public void reshape(int numberOfFootsteps)
   {
      int size = 2 * numberOfFootsteps;
      quadraticTerm.reshape(size, size);
      linearTerm.reshape(size, 1);
   }

}
