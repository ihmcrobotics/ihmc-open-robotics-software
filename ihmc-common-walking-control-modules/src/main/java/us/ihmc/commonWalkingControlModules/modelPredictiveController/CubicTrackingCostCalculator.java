package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CubicTrackingCommand;

public class CubicTrackingCostCalculator
{
   public static boolean calculateTrackingObjective(DMatrixRMaj costHessianToPack, DMatrixRMaj costGradientToPack, CubicTrackingCommand objective)
   {
      double t = objective.getDuration();
      double t2 = t * t;
      double t3 = t * t2;
      double t4 = t * t3;
      double t5 = t * t4;
      double t6 = t * t5;
      double t7 = t * t6;

      double a3 = objective.getStartValue();
      double a2 = objective.getStartRate();
      double a1 = 3.0 / t2 * (objective.getFinalValue() - a3) - (objective.getFinalRate() + 2.0 * a2) / t;
      double a0 = (objective.getFinalRate() + a2) / t2 - 2.0 / t3 * (objective.getFinalValue() - a3);

      int startIndex = objective.getStartIndex();

      costHessianToPack.set(startIndex, startIndex, t7 / 7.0);
      costHessianToPack.set(startIndex + 1, startIndex, t6 / 6.0);
      costHessianToPack.set(startIndex + 2, startIndex, t5 / 5.0);
      costHessianToPack.set(startIndex + 3, startIndex, t4 / 4.0);

      costHessianToPack.set(startIndex, startIndex + 1, t6 / 6.0);
      costHessianToPack.set(startIndex + 1, startIndex + 1, t5 / 5.0);
      costHessianToPack.set(startIndex + 2, startIndex + 1, t4 / 4.0);
      costHessianToPack.set(startIndex + 3, startIndex + 1, t3 / 3.0);

      costHessianToPack.set(startIndex, startIndex + 2, t5 / 5.0);
      costHessianToPack.set(startIndex + 1, startIndex + 2, t4 / 4.0);
      costHessianToPack.set(startIndex + 2, startIndex + 2, t3 / 3.0);
      costHessianToPack.set(startIndex + 3, startIndex + 2, t2 / 2.0);

      costHessianToPack.set(startIndex, startIndex + 3, t4 / 4.0);
      costHessianToPack.set(startIndex + 1, startIndex + 3, t3 / 3.0);
      costHessianToPack.set(startIndex + 2, startIndex + 3, t2 / 2.0);
      costHessianToPack.set(startIndex + 3, startIndex + 3, t);

      costGradientToPack.set(startIndex, 0, a0 * t7 / 7.0 + a1 * t6 / 6.0 + a2 * t5 / 5.0 + a3 * t4 / 4.0);
      costGradientToPack.set(startIndex + 2, 0, a0 * t6 / 6.0 + a1 * t5 / 5.0 + a2 * t4 / 4.0 + a3 * t3 / 3.0);
      costGradientToPack.set(startIndex + 3, 0, a0 * t5 / 5.0 + a1 * t4 / 4.0 + a2 * t3 / 3.0 + a3 * t2 / 2.0);
      costGradientToPack.set(startIndex + 4, 0, a0 * t4 / 4.0 + a1 * t3 / 3.0 + a2 * t2 / 2.0 + a3 * t);

      return false;
   }

}
