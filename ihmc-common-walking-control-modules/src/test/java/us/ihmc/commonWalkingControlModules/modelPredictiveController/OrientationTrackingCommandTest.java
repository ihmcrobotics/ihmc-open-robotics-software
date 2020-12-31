package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrackingCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.assertEquals;

public class OrientationTrackingCommandTest
{
   @Test
   public void testCommandOptimize()
   {
      double gravityZ = -9.81;
      double mass = 1.5;
      double dt = 1e-3;

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      CoMMPCQPSolver solver = new CoMMPCQPSolver(indexHandler, dt, mass, gravityZ, new YoRegistry("test"));


      double startYawValue = -2.0;
      double startYawRate = 11.0;
      double endYawValue = 3.0;
      double endYawRate = -9.0;

      double startPitchValue = -1.0;
      double startPitchRate = 12.0;
      double endPitchValue = 1.5;
      double endPitchRate = -10.0;

      double startRollValue = -2.5;
      double startRollRate = 13.0;
      double endRollValue = 2.5;
      double endRollRate = -11.0;

      FrameVector3D startRate = new FrameVector3D();
      startRate.set(startRollRate, startPitchRate, startYawRate);
      FrameVector3D endRate = new FrameVector3D();
      endRate.set(endRollRate, endPitchRate, endYawRate);

      FrameQuaternion startOrientation = new FrameQuaternion();
      FrameQuaternion endOrientation = new FrameQuaternion();
      startOrientation.setYawPitchRoll(startYawValue, startPitchValue, startRollValue);
      endOrientation.setYawPitchRoll(endYawValue, endPitchValue, endRollValue);

      indexHandler.initialize((i) -> 4, 1);

      double duration = 0.7;

      int startYawIndex = indexHandler.getYawCoefficientsStartIndex(0);
      int startPitchIndex = indexHandler.getPitchCoefficientsStartIndex(0);
      int startRollIndex = indexHandler.getRollCoefficientsStartIndex(0);

      OrientationTrackingCommand command = new OrientationTrackingCommand();
      command.setStartOrientation(startOrientation);
      command.setStartAngularRate(startRate);
      command.setFinalOrientation(endOrientation);
      command.setFinalAngularRate(endRate);
      command.setSegmentDuration(duration);
      command.setSegmentNumber(0, indexHandler);
      command.setWeight(100.0);

      double regularization = 1e-5;
      solver.initialize();
      solver.submitOrientationTrackingCommand(command);
      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);
      solver.setOrientationCoefficientRegularization(regularization);

      solver.solve();

      DMatrixRMaj solution = solver.getSolution();


      double a3Yaw = startYawValue;
      double a2Yaw = startYawRate;
      double a1Yaw = 3.0 / Math.pow(duration, 2) * (endYawValue - a3Yaw) - (endYawRate + 2.0 * a2Yaw) / duration;
      double a0Yaw = (endYawRate + a2Yaw) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endYawValue - a3Yaw);

      double a3Pitch = startPitchValue;
      double a2Pitch = startPitchRate;
      double a1Pitch = 3.0 / Math.pow(duration, 2) * (endPitchValue - a3Pitch) - (endPitchRate + 2.0 * a2Pitch) / duration;
      double a0Pitch = (endPitchRate + a2Pitch) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endPitchValue - a3Pitch);

      double a3Roll = startRollValue;
      double a2Roll = startRollRate;
      double a1Roll = 3.0 / Math.pow(duration, 2) * (endRollValue - a3Roll) - (endRollRate + 2.0 * a2Roll) / duration;
      double a0Roll = (endRollRate + a2Roll) / Math.pow(duration, 2) - 2.0 / Math.pow(duration, 3) * (endRollValue - a3Roll);

      for (double time = 0.0; time <= duration; time += 0.001)
      {
         double assembledYawValue = solution.get(startYawIndex, 0) * MathTools.pow(time, 3);
         assembledYawValue += solution.get(startYawIndex + 1, 0) * MathTools.pow(time, 2);
         assembledYawValue += solution.get(startYawIndex + 2, 0) * MathTools.pow(time, 1);
         assembledYawValue += solution.get(startYawIndex + 3, 0);

         double assembledPitchValue = solution.get(startPitchIndex, 0) * MathTools.pow(time, 3);
         assembledPitchValue += solution.get(startPitchIndex + 1, 0) * MathTools.pow(time, 2);
         assembledPitchValue += solution.get(startPitchIndex + 2, 0) * MathTools.pow(time, 1);
         assembledPitchValue += solution.get(startPitchIndex + 3, 0);

         double assembledRollValue = solution.get(startRollIndex, 0) * MathTools.pow(time, 3);
         assembledRollValue += solution.get(startRollIndex + 1, 0) * MathTools.pow(time, 2);
         assembledRollValue += solution.get(startRollIndex + 2, 0) * MathTools.pow(time, 1);
         assembledRollValue += solution.get(startRollIndex + 3, 0);

         double assembledYawRate = solution.get(startYawIndex, 0) * 3.0 * MathTools.pow(time, 2);
         assembledYawRate += solution.get(startYawIndex + 1, 0) * 2.0 * time;
         assembledYawRate += solution.get(startYawIndex + 2, 0);

         double assembledPitchRate = solution.get(startPitchIndex, 0) * 3.0 * MathTools.pow(time, 2);
         assembledPitchRate += solution.get(startPitchIndex + 1, 0) * 2.0 * time;
         assembledPitchRate += solution.get(startPitchIndex + 2, 0);

         double assembledRollRate = solution.get(startRollIndex, 0) * 3.0 * MathTools.pow(time, 2);
         assembledRollRate += solution.get(startRollIndex + 1, 0) * 2.0 * time;
         assembledRollRate += solution.get(startRollIndex + 2, 0);


         double expectedYawValue = a0Yaw * Math.pow(time, 3) + a1Yaw * time * time + a2Yaw * time + a3Yaw;
         double expectedYawRate = a0Yaw * 3.0 * Math.pow(time, 2) + a1Yaw * 2.0 *  time + a2Yaw;
         double expectedPitchValue = a0Pitch * Math.pow(time, 3) + a1Pitch * time * time + a2Pitch * time + a3Pitch;
         double expectedPitchRate = a0Pitch * 3.0 * Math.pow(time, 2) + a1Pitch * 2.0 *  time + a2Pitch;
         double expectedRollValue = a0Roll * Math.pow(time, 3) + a1Roll * time * time + a2Roll * time + a3Roll;
         double expectedRollRate = a0Roll * 3.0 * Math.pow(time, 2) + a1Roll * 2.0 *  time + a2Roll;

         assertEquals("Failed at time " + time, expectedYawValue, assembledYawValue, 5e-1);
//         assertEquals("Failed at time " + time, expectedYawRate, assembledYawRate, 5e-1);
         assertEquals("Failed at time " + time, expectedPitchValue, assembledPitchValue, 5e-1);
//         assertEquals("Failed at time " + time, expectedPitchRate, assembledPitchRate, 5e-1);
         assertEquals("Failed at time " + time, expectedRollValue, assembledRollValue, 5e-1);
//         assertEquals("Failed at time " + time, expectedRollRate, assembledRollRate, 5e-1);
      }
   }
}
