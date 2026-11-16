package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

import static org.junit.jupiter.api.Assertions.*;

public class PoseErrorEKFTest
{
   private static final long MONO = 10_000_000_000L;
   private static final long STEP = 5_000_000L;

   private static EKFPoseFusionParameters parameters(double processStandardDeviation,
                                                          double measurementStandardDeviation,
                                                          double maximumPositionInnovation,
                                                          double maximumOrientationInnovation)
   {
      EKFPoseFusionParameters parameters = new EKFPoseFusionParameters();
      parameters.setPositionProcessStandardDeviation(processStandardDeviation);
      parameters.setOrientationProcessStandardDeviation(processStandardDeviation);
      parameters.setPositionMeasurementStandardDeviation(measurementStandardDeviation);
      parameters.setOrientationMeasurementStandardDeviation(measurementStandardDeviation);
      parameters.setMaximumPositionInnovation(maximumPositionInnovation);
      parameters.setMaximumOrientationInnovation(maximumOrientationInnovation);
      return parameters;
   }

   @Test
   public void statisticalUpdateIsNotTickClippedAndJosephCovarianceRemainsSymmetricAndNonnegative()
   {
      PoseErrorEKF filter = new PoseErrorEKF(parameters(1, 0.001, 1, 1), new YoRegistry("test"));
      RigidBodyTransform result = new RigidBodyTransform();
      filter.update(MONO, pose(0), pose(0), result);
      filter.update(MONO + STEP, pose(0), pose(0.1), result);
      assertTrue(result.getTranslationX() > 0.09, "The posterior must not be clipped to one estimator tick's output allowance");
      for (int tick = 2; tick < 200; tick++)
         filter.update(MONO + tick * STEP, pose(0), pose(0.1), result);
      DMatrixRMaj covariance = filter.getCovariance();
      for (int row = 0; row < 6; row++)
      {
         assertTrue(Double.isFinite(covariance.get(row, row)) && covariance.get(row, row) >= 0);
         for (int column = 0; column < 6; column++)
            assertEquals(covariance.get(row, column), covariance.get(column, row), 1e-12);
      }
   }

   @Test
   public void retainsRobotPredictionDuringMeasurementDropoutAndRejectsRepeatedTimestamps()
   {
      PoseErrorEKF filter = new PoseErrorEKF(parameters(0.1, 0.1, 0.2, 0.25), new YoRegistry("test"));
      RigidBodyTransform pose = new RigidBodyTransform();
      RigidBodyTransform result = new RigidBodyTransform();
      filter.update(1_000_000_000L, pose, pose, result);
      pose.getTranslation().setX(0.1);
      filter.update(1_020_000_000L, pose, null, result);
      assertEquals(0.1, result.getTranslationX(), 1.e-9);
      assertThrows(IllegalArgumentException.class, () -> filter.update(1_020_000_000L, pose, null, result));
   }

   @Test
   @SuppressWarnings("unchecked")
   public void innovationLimitsAreYoVariablesThatTakeEffectAtRuntime()
   {
      YoRegistry registry = new YoRegistry("test");
      PoseErrorEKF filter = new PoseErrorEKF(parameters(0.1, 0.1, 0.2, 0.25), registry);
      YoEnum<PoseErrorEKF.State> state = (YoEnum<PoseErrorEKF.State>) registry.findVariable("ekfState");
      RigidBodyTransform result = new RigidBodyTransform();
      filter.update(MONO, pose(0), pose(0), result);
      assertEquals(PoseErrorEKF.State.MEASUREMENT_WORLD_ALIGNED, state.getEnumValue());

      filter.update(MONO + STEP, pose(0), pose(0.3), result); // Past the position innovation limit.
      assertEquals(PoseErrorEKF.State.MEASUREMENT_REJECTED, state.getEnumValue());
      assertEquals(0.3, ((YoDouble) registry.findVariable("ekfPositionInnovation")).getValue(), 1.0e-9);

      ((YoDouble) registry.findVariable("ekfMaximumPositionInnovation")).set(0.5);
      ((YoDouble) registry.findVariable("ekfMaximumNormalizedInnovationSquared")).set(Double.POSITIVE_INFINITY);
      filter.update(MONO + 2 * STEP, pose(0), pose(0.3), result);
      assertEquals(PoseErrorEKF.State.MEASUREMENT_ACCEPTED, state.getEnumValue());
      assertEquals(1, ((YoLong) registry.findVariable("ekfAcceptedMeasurementCount")).getValue());
      assertEquals(1, ((YoLong) registry.findVariable("ekfRejectedMeasurementCount")).getValue());

      filter.update(MONO + 3 * STEP, pose(0), null, result);
      assertEquals(PoseErrorEKF.State.PREDICTING, state.getEnumValue());
   }

   private static RigidBodyTransform pose(double x)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().setX(x);
      return pose;
   }
}
