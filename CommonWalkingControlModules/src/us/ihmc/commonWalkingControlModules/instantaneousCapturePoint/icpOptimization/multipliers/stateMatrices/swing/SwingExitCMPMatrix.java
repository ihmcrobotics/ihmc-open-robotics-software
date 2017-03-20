package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SwingExitCMPMatrix extends DenseMatrix64F
{
   private final DoubleYoVariable upcomingDoubleSupportSplitRatio;
   private final DoubleYoVariable exitCMPRatio;

   private final DoubleYoVariable endOfSplineTime;

   public SwingExitCMPMatrix(DoubleYoVariable upcomingDoubleSupportSplitRatio, DoubleYoVariable exitCMPRatio, DoubleYoVariable endOfSplineTime)
   {
      super(4, 1);

      this.upcomingDoubleSupportSplitRatio = upcomingDoubleSupportSplitRatio;
      this.exitCMPRatio = exitCMPRatio;

      this.endOfSplineTime = endOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      compute(doubleSupportDurations.get(1).getDoubleValue(), doubleSupportDurations.get(0).getDoubleValue(), singleSupportDurations.get(0).getDoubleValue(), omega0);
   }

   public void compute(double upcomingDoubleSupportDuration, double currentDoubleSupportDuration, double singleSupportDuration, double omega0)
   {
      double currentStepDuration = currentDoubleSupportDuration + singleSupportDuration;
      double upcomingInitialDoubleSupportTime = upcomingDoubleSupportSplitRatio.getDoubleValue() * upcomingDoubleSupportDuration;
      double timeSpentOnCurentExitCMP = exitCMPRatio.getDoubleValue() * currentStepDuration;

      double duration = endOfSplineTime.getDoubleValue() - singleSupportDuration + timeSpentOnCurentExitCMP - upcomingInitialDoubleSupportTime;
      double projection = Math.exp(omega0 * duration);

      zero();

      set(2, 0, 1.0 - projection);
      set(3, 0, -omega0 * projection);
   }
}
