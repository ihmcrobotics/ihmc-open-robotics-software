package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferInitialICPProjectionMatrix extends DenseMatrix64F
{
   public TransferInitialICPProjectionMatrix()
   {
      super(4, 1);
   }

   public void reset()
   {
      zero();
   }

   public void compute(double omega0)
   {
      set(0, 0, 1);
      set(1, 0, omega0);
   }
}
