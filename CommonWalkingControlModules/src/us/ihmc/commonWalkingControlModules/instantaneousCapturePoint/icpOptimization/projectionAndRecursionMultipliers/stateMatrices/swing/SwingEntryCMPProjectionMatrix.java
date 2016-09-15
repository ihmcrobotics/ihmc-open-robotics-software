package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class SwingEntryCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable startOfSplineTime;

   public SwingEntryCMPProjectionMatrix(DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         DoubleYoVariable startOfSplineTime)
   {
      super(4, 1);

      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      this.startOfSplineTime = startOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      this.compute(doubleSupportDurations.get(0).getDoubleValue(), singleSupportDurations.get(0).getDoubleValue(), omega0);
   }

   public void compute(double doubleSupportDuration, double singleSupportDuration, double omega0)
   {
      zero();

      double stepDuration = doubleSupportDuration + singleSupportDuration;

      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDuration;
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

      double projectionDuration = startOfSplineTime.getDoubleValue() + endOfDoubleSupportDuration - timeSpentOnEntryCMP;

      double projection = Math.exp(omega0 * projectionDuration);

      set(0, 0, 1.0 - projection);
      set(1, 0, -omega0 * projection);
   }

}
