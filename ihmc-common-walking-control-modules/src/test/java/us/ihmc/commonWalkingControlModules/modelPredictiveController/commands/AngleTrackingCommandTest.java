package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMMPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCIndexHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertEquals;

public class AngleTrackingCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      double gravityZ = -9.81;
      double mass = 1.5;
      double dt = 1e-3;
      double omega = 3.0;

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, mass, gravityZ, new YoRegistry("test"));


      double startValue = -5.0;
      double startRate = 11.0;
      double endValue = 7.0;
      double endRate = -9.0;


      indexHandler.initialize((i) -> 4, 1);

      double duration = 0.7;
      int startIndex = indexHandler.getOrientationCoefficientsStartIndex(0);


      AngleTrackingCommand command = new AngleTrackingCommand();
      command.setStartValue(startValue);
      command.setStartRate(startRate);
      command.setFinalValue(endValue);
      command.setFinalRate(endRate);
      command.setDuration(duration);
      command.setStartIndex(startIndex);
      command.setWeight(100.0);
      command.setOmega(omega);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitCubicTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);
      solver.setOrientationCoefficientRegularization(regularization);

      solver.solve();

      DMatrixRMaj solution = solver.getSolution();


      double a3 = startValue;
      double a2 = startRate;
      double a1 = 3.0 / Math.pow(duration, 2) * (endValue - a3) - (endRate + 2.0 * a2) / duration;
      double a0 = (endRate + a2) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endValue - a3);


//      assertEquals(a0, solution.get(startIndex, 0), 1e-5);
//      assertEquals(a1, solution.get(startIndex + 1, 0), 1e-5);
//      assertEquals(a2, solution.get(startIndex + 2, 0), 1e-5);
//      assertEquals(a3, solution.get(startIndex + 3, 0), 1e-5);

      for (double time = 0.0; time <= duration; time += 0.001)
      {
         double assembledValue = 0.0;
         double assembledRate = 0.0;
         if (MPCIndexHandler.includeExponentialInOrientation)
         {
            assembledValue += solution.get(startIndex, 0) * Math.exp(omega * time);
            assembledValue += solution.get(startIndex + 1, 0) * Math.exp(-omega * time);

            assembledRate += solution.get(startIndex, 0) * omega * Math.exp(omega * time);
            assembledRate += solution.get(startIndex + 1, 0) * -omega * Math.exp(-omega * time);

            startIndex += 2;
         }
         assembledValue += solution.get(startIndex, 0) * MathTools.pow(time, 3);
         assembledValue += solution.get(startIndex + 1, 0) * MathTools.pow(time, 2);
         assembledValue += solution.get(startIndex + 2, 0) * MathTools.pow(time, 1);
         assembledValue += solution.get(startIndex + 3, 0);


         assembledRate += solution.get(startIndex, 0) * 3.0 * MathTools.pow(time, 2);
         assembledRate += solution.get(startIndex + 1, 0) * 2.0 * time;
         assembledRate += solution.get(startIndex + 2, 0);

         double expectedValue = a0 * Math.pow(time, 3) + a1 * time * time + a2 * time + a3;
         double expectedRate = a0 * 3.0 * Math.pow(time, 2) + a1 * 2.0 *  time + a2;

         assertEquals("Failed at time " + time, expectedValue, assembledValue, 5e-1);
         assertEquals("Failed at time " + time, expectedRate, assembledRate, 5e-1);
      }
   }
}
