package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.AngleTrackingCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCIndexHandler;
import us.ihmc.matrixlib.MatrixTools;

public class AngleTrackingCostCalculator
{
   public static boolean calculateTrackingObjective(DMatrixRMaj costHessianToPack, DMatrixRMaj costGradientToPack, AngleTrackingCommand objective)
   {
      double t = objective.getDuration();
      double t2 = t * t;
      double t3 = t * t2;
      double t4 = t * t3;
      double t5 = t * t4;
      double t6 = t * t5;
      double t7 = t * t6;

      double w = objective.getOmega();
      double w2 = w * w;
      double w3 = w * w2;
      double w4 = w * w3;

      int startIndex = objective.getStartIndex();

      double exponential = Math.exp(w * t);
      double negxponential = 1.0 / exponential;

      double a3 = objective.getStartValue();
      double a2 = objective.getStartRate();
      double a1 = 3.0 / t2 * (objective.getFinalValue() - a3) - (objective.getFinalRate() + 2.0 * a2) / t;
      double a0 = (objective.getFinalRate() + a2) / t2 - 2.0 / t3 * (objective.getFinalValue() - a3);

      if (ContinuousMPCIndexHandler.includeExponentialInOrientation)
      {
         double h00 = (exponential * exponential - 1.0) / (2.0 * w);
         double h10 = t;
         double h20 = (exponential * (w3 * t3 - 3.0 * w2 * t2 + 6 * w * t - 6) + 6) / w4;
         double h30 = (exponential * (w2 * t2 - 2.0 * w * t + 2) - 2.0) / w3;
         double h40 = (exponential * (w * t - 1) + 1) / w2;
         double h50 = (exponential - 1.0) / w;

         double h11 = -(negxponential * negxponential - 1.0) / (2.0 * w);
         double h21 = (negxponential * (w3 * t3 + 3.0 * w2 * t2 + 6 * w * t + 6) - 6) / w4;
         double h31 = (negxponential * (w2 * t2 + 2 * w * t + 2) - 2) / w3;
         double h41 = (negxponential * (w * t + 1) - 1) / w2;
         double h51 = -(negxponential - 1) / w;

         costHessianToPack.add(startIndex, startIndex, h00);
         costHessianToPack.add(startIndex + 1, startIndex, h10);
         costHessianToPack.add(startIndex + 2, startIndex, h20);
         costHessianToPack.add(startIndex + 3, startIndex, h30);
         costHessianToPack.add(startIndex + 4, startIndex, h40);
         costHessianToPack.add(startIndex + 5, startIndex, h50);

         costHessianToPack.add(startIndex, startIndex + 1, h10);
         costHessianToPack.add(startIndex + 1, startIndex + 1, h11);
         costHessianToPack.add(startIndex + 2, startIndex + 1, h21);
         costHessianToPack.add(startIndex + 3, startIndex + 1, h31);
         costHessianToPack.add(startIndex + 4, startIndex + 1, h41);
         costHessianToPack.add(startIndex + 5, startIndex + 1, h51);

         costHessianToPack.add(startIndex, startIndex + 2, h20);
         costHessianToPack.add(startIndex + 1, startIndex + 2, h21);

         costHessianToPack.add(startIndex, startIndex + 3, h30);
         costHessianToPack.add(startIndex + 1, startIndex + 3, h31);

         costHessianToPack.add(startIndex, startIndex + 4, h40);
         costHessianToPack.add(startIndex + 1, startIndex + 4, h41);

         costHessianToPack.add(startIndex, startIndex + 5, h50);
         costHessianToPack.add(startIndex + 1, startIndex + 5, h51);

         double g0 = a0 * h20 + a1 * h30 + a2 * h40 + a3 * h50;
         double g1 = a0 * h21 + a1 * h31 + a2 * h41 + a3 * h51;

         costGradientToPack.add(startIndex, 0, -g0);
         costGradientToPack.add(startIndex + 1, 0, -g1);

         startIndex += 2;
      }

      double h22 = t7 / 7.0;
      double h32 = t6 / 6.0;
      double h42 = t5 / 5.0;
      double h52 = t4 / 4.0;

      double h33 = h42;
      double h43 = h52;
      double h53 = t3 / 3.0;

      double h44 = h53;
      double h54 = t2 / 2.0;

      double h55 = t;

      costHessianToPack.add(startIndex, startIndex, h22);
      costHessianToPack.add(startIndex + 1, startIndex, h32);
      costHessianToPack.add(startIndex + 2, startIndex, h42);
      costHessianToPack.add(startIndex + 3, startIndex, h52);

      costHessianToPack.add(startIndex, startIndex + 1, h32);
      costHessianToPack.add(startIndex + 1, startIndex + 1, h33);
      costHessianToPack.add(startIndex + 2, startIndex + 1, h43);
      costHessianToPack.add(startIndex + 3, startIndex + 1, h53);

      costHessianToPack.add(startIndex, startIndex + 2, h42);
      costHessianToPack.add(startIndex + 1, startIndex + 2, h43);
      costHessianToPack.add(startIndex + 2, startIndex + 2, h44);
      costHessianToPack.add(startIndex + 3, startIndex + 2, h54);

      costHessianToPack.add(startIndex, startIndex + 3, h52);
      costHessianToPack.add(startIndex + 1, startIndex + 3, h53);
      costHessianToPack.add(startIndex + 2, startIndex + 3, h54);
      costHessianToPack.add(startIndex + 3, startIndex + 3, h55);

      double g2 = a0 * h22 + a1 * h32 + a2 * h42 + a3 * h52;
      double g3 = a0 * h32 + a1 * h33 + a2 * h43 + a3 * h53;
      double g4 = a0 * h42 + a1 * h43 + a2 * h44 + a3 * h54;
      double g5 = a0 * h52 + a1 * h53 + a2 * h54 + a3 * h55;

      costGradientToPack.add(startIndex, 0, -g2);
      costGradientToPack.add(startIndex + 1, 0, -g3);
      costGradientToPack.add(startIndex + 2, 0, -g4);
      costGradientToPack.add(startIndex + 3, 0, -g5);

      if (MatrixTools.containsNaN(costHessianToPack))
         throw new RuntimeException("error");

      return true;
   }

}
