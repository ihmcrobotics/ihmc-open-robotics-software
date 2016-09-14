package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class SwingEntryCMPProjectionMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   private final DoubleYoVariable startOfSplineTime;

   public SwingEntryCMPProjectionMatrix(DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitRatio, DoubleYoVariable exitCMPDurationInPercentOfStepTime,
         DoubleYoVariable startOfSplineTime)
   {
      super(4, 1);

      this.omega = omega;
      this.doubleSupportSplitRatio = doubleSupportSplitRatio;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;

      this.startOfSplineTime = startOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations)
   {
      this.compute(doubleSupportDurations.get(0).getDoubleValue(), singleSupportDurations.get(0).getDoubleValue());
   }

   public void compute(double doubleSupportDuration, double singleSupportDuration)
   {
      zero();

      double stepDuration = doubleSupportDuration + singleSupportDuration;

      double endOfDoubleSupportDuration = (1.0 - doubleSupportSplitRatio.getDoubleValue()) * doubleSupportDuration;
      double timeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * stepDuration;

      double projectionDuration = startOfSplineTime.getDoubleValue() + endOfDoubleSupportDuration - timeSpentOnEntryCMP;

      double projection = Math.exp(omega.getDoubleValue() * projectionDuration);

      set(0, 0, 1.0 - projection);
      set(1, 0, -omega.getDoubleValue() * projection);
   }

}
