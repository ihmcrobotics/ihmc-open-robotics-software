package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class TransferStateEndRecursionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;

   public TransferStateEndRecursionMatrix(DoubleYoVariable omega)
   {
      super(4, 1);

      this.omega = omega;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations)
   {
      compute(doubleSupportDurations.get(0).getDoubleValue());
   }

   public void compute(double doubleSupportDuration)
   {
      double stateRecursion = Math.exp(-omega.getDoubleValue() * doubleSupportDuration);

      set(0, 0, stateRecursion);
      set(1, 0, omega.getDoubleValue() * stateRecursion);
      set(2, 0, 1.0);
      set(3, 0, omega.getDoubleValue());
   }

}

