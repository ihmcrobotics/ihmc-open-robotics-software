package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class SwingStateEndRecursionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable defaultDoubleSupportSplitRatio;
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;
   private final DoubleYoVariable startOfSplineTime;
   private final DoubleYoVariable endOfSplineTime;
   private final DoubleYoVariable totalTrajectoryTime;

   public SwingStateEndRecursionMatrix(DoubleYoVariable defaultDoubleSupportSplitRatio, DoubleYoVariable upcomingDoubleSupportSplitRatio,
         DoubleYoVariable startOfSplineTime, DoubleYoVariable endOfSplineTime, DoubleYoVariable totalTrajectoryTime)
   {
      super(4, 1);

      this.defaultDoubleSupportSplitRatio = defaultDoubleSupportSplitRatio;
      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;
      this.startOfSplineTime = startOfSplineTime;
      this.endOfSplineTime = endOfSplineTime;
      this.totalTrajectoryTime = totalTrajectoryTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double omega0, boolean useInitialICP)
   {
      double upcomingDoubleSupportDuration = doubleSupportDurations.get(1).getDoubleValue();
      double currentDoubleSupportDuration = doubleSupportDurations.get(0).getDoubleValue();

      compute(upcomingDoubleSupportDuration, currentDoubleSupportDuration, singleSupportDurations.get(0).getDoubleValue(), omega0, useInitialICP);
   }

   public void compute(double upcomingDoubleSupportDuration, double currentDoubleSupportDuration, double singleSupportDuration, double omega0, boolean useInitialICP)
   {
      double stepDuration = currentDoubleSupportDuration + singleSupportDuration;

      double endOfCurrentDoubleSupportDuration = (1.0 - defaultDoubleSupportSplitRatio.getDoubleValue()) * currentDoubleSupportDuration;
      double upcomingInitialDoubleSupportDuration = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;

      double lastSegmentDuration = totalTrajectoryTime.getDoubleValue() - endOfSplineTime.getDoubleValue();
      double stateRecursionToEnd = Math.exp(-omega0 * lastSegmentDuration);

      double recursionTimeToInitial = upcomingInitialDoubleSupportDuration + endOfCurrentDoubleSupportDuration + startOfSplineTime.getDoubleValue() - stepDuration;
      double stateRecursionToStart = Math.exp(omega0 * recursionTimeToInitial);

      zero();
      if (!useInitialICP)
      {
         set(0, 0, stateRecursionToStart);
         set(1, 0, omega0 * stateRecursionToStart);
      }
      set(2, 0, stateRecursionToEnd);
      set(3, 0, omega0 * stateRecursionToEnd);
   }
}

