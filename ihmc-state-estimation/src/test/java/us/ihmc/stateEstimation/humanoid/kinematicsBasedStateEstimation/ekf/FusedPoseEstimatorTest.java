package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.UUID;

import static org.junit.jupiter.api.Assertions.*;

public class FusedPoseEstimatorTest
{
   private static final long WALL = 1_793_212_000_000_000_000L, MONO = 10_000_000_000L, STEP = 5_000_000L;

   private static RigidBodyTransform pose(double x)
   {
      var pose = new RigidBodyTransform();
      pose.getTranslation().setX(x);
      return pose;
   }

   /** A pose measurement in the world named {@code frameId}, exposed at {@code exposureTime}. */
   static StampedPosePacket measurement(String frameId, long exposureTime, RigidBodyTransform pose)
   {
      var packet = new StampedPosePacket();
      packet.getFrameId().append(frameId);
      packet.setTimestamp(exposureTime);
      packet.getPose().set(pose);
      return packet;
   }

   @Test
   public void fusesKinematicsWithoutMutatingInputsAndRetainsOffsetDuringDropoutAndNewWorld()
   {
      var estimator = new FusedPoseEstimator();
      String frame = UUID.randomUUID().toString();
      var result = new RigidBodyTransform();
      for (int i = 0; i <= 200; i++)
      {
         var kinematic = pose(i * .0004);
         estimator.update(WALL + i * STEP, MONO + i * STEP, kinematic, i % 6 == 0 ? measurement(frame, WALL + i * STEP, pose(0)) : null, result);
         assertEquals(i * .0004, kinematic.getTranslationX(), 1e-12);
      }
      assertTrue(result.getTranslationX() < .06, "Pose measurements should reduce the kinematic drift");
      double offset = result.getTranslationX() - .08;
      for (int i = 201; i <= 250; i++)
      {
         estimator.update(WALL + i * STEP, MONO + i * STEP, pose(i * .0004), null, result);
         assertEquals(offset + i * .0004, result.getTranslationX(), 1e-10);
      }
      // The new measurement world is 10 m away: its first measurement realigns to the retained estimate.
      String nextFrame = UUID.randomUUID().toString();
      var realigned = new RigidBodyTransform();
      estimator.update(WALL + 251 * STEP, MONO + 251 * STEP, pose(251 * .0004), measurement(nextFrame, WALL + 251 * STEP, pose(10)), realigned);
      assertEquals(result.getTranslationX() + .0004, realigned.getTranslationX(), 1e-10);
   }

   @Test
   public void compensatesDelayedExposureWithKinematicMotionAndRejectsUnbracketedInput()
   {
      var estimator = new FusedPoseEstimator();
      String frame = UUID.randomUUID().toString();
      var result = new RigidBodyTransform();
      for (int i = 0; i < 100; i++)
      {
         int exposure = i - 4;
         var measurement = i >= 4 && i % 6 == 0 ? measurement(frame, WALL + exposure * STEP, pose(exposure * .001)) : null;
         estimator.update(WALL + i * STEP, MONO + i * STEP, pose(i * .001), measurement, result);
         assertEquals(i * .001, result.getTranslationX(), 1e-10);
      }
      estimator.update(WALL + 100 * STEP, MONO + 100 * STEP, pose(.1), measurement(frame, WALL + 101 * STEP, pose(9)), result);
      assertEquals(.1, result.getTranslationX(), 1e-10);
   }

   @Test
   public void oneMeasurementKeepsMovingTheOutputBetweenMeasurementsWithoutRepeatedEkfUpdates()
   {
      var estimator = new FusedPoseEstimator();
      String frame = UUID.randomUUID().toString();
      double previous = 0;
      var result = new RigidBodyTransform();
      for (int i = 0; i <= 200; i++)
      {
         long exposure = WALL + i * 1_000_000L;
         var measurement = i == 0 ? measurement(frame, exposure, pose(0)) : i == 33 ? measurement(frame, exposure, pose(.1)) : null;
         estimator.update(exposure, MONO + i * 1_000_000L, pose(0), measurement, result);
         assertTrue(result.getTranslationX() - previous <= .0005 + 1e-12);
         if (i >= 33 && i <= 62)
            assertEquals(.0005, result.getTranslationX() - previous, 1e-10, "The correction must continue on ticks without a measurement");
         if (i == 62)
            assertEquals(.015, result.getTranslationX(), 1e-10);
         previous = result.getTranslationX();
      }
      assertTrue(previous > .05 && previous < .06);
      assertTrue(estimator.getStatus().contains("accepted=1, rejected=0"), estimator.getStatus());
   }

   @Test
   public void millisecondQuantizedWallTimeDoesNotDisableHighRateFusion()
   {
      var estimator = new FusedPoseEstimator();
      String frame = UUID.randomUUID().toString();
      var result = new RigidBodyTransform();
      estimator.update(WALL, MONO, pose(0), measurement(frame, WALL, pose(0)), result);
      estimator.update(WALL, MONO + 500_000L, pose(.001), null, result);
      estimator.update(WALL + 1_000_000L, MONO + 1_000_000L, pose(.002), measurement(frame, WALL + 1_000_000L, pose(0)), result);
      assertFalse(estimator.getStatus().contains("CLOCK FAULT"));
   }

   @Test
   public void clockDiscontinuityLatchesPredictionOnly()
   {
      var estimator = new FusedPoseEstimator();
      String frame = UUID.randomUUID().toString();
      var predicted = new RigidBodyTransform();
      estimator.update(WALL, MONO, pose(0), measurement(frame, WALL, pose(0)), predicted);
      estimator.update(WALL + 10 * STEP, MONO + STEP, pose(.01), null, predicted);
      estimator.update(WALL + 11 * STEP, MONO + 2 * STEP, pose(.02), measurement(frame, WALL + 11 * STEP, pose(-.1)), predicted);
      assertEquals(.02, predicted.getTranslationX(), 1e-10);
      assertTrue(estimator.getStatus().contains("CLOCK FAULT"));
   }
}
