package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class SwingInitialICPMatrix extends DenseMatrix64F
{
   private final YoDouble startOfSplineTime;

   private final boolean blendFromInitial;
   private final double minimumBlendingTime;

   public SwingInitialICPMatrix(YoDouble startOfSplineTime, boolean blendFromInitial, double minimumBlendingTime)
   {
      super(4, 1);

      this.startOfSplineTime = startOfSplineTime;
      this.blendFromInitial = blendFromInitial;
      this.minimumBlendingTime = minimumBlendingTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(double omega0)
   {
      if (blendFromInitial)
      {
         zero();

         if (startOfSplineTime.getDoubleValue() < minimumBlendingTime)
         { // haven't phased in completely yet, so its a combination of both recursion and projection
            double phaseAtStart = startOfSplineTime.getDoubleValue() / minimumBlendingTime;

            double recursionMultiplier = 0.0;
            double projectionMultiplier = Math.exp(omega0 * startOfSplineTime.getDoubleValue());

            double projection = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, phaseAtStart);

            set(0, 0, projection);
            set(1, 0, omega0 * projection);
         }
      }
      else
      {
         // project forward from the initial ICP state
         double projection = Math.exp(omega0 * startOfSplineTime.getDoubleValue());

         zero();
         set(0, 0, projection);
         set(1, 0, omega0 * projection);
      }
   }
}

