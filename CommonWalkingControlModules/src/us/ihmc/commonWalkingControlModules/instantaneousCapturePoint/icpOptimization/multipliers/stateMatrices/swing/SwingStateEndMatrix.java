package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SwingStateEndMatrix extends DenseMatrix64F
{
   private final List<DoubleYoVariable> swingSplitFractions;

   private final DoubleYoVariable endOfSplineTime;

   public SwingStateEndMatrix(List<DoubleYoVariable> swingSplitFractions, DoubleYoVariable endOfSplineTime)
   {
      super(4, 1);

      this.swingSplitFractions = swingSplitFractions;

      this.endOfSplineTime = endOfSplineTime;
   }

   public void reset()
   {
      zero();
   }

   public void compute(List<DoubleYoVariable> singleSupportDurations, double omega0)
   {
      double currentSwingOnEntryCMP = swingSplitFractions.get(0).getDoubleValue() * singleSupportDurations.get(0).getDoubleValue();

      double duration = endOfSplineTime.getDoubleValue() - currentSwingOnEntryCMP;
      double projection = Math.exp(omega0 * duration);

      zero();

      set(2, 0, projection);
      set(3, 0, omega0 * projection);
   }
}

