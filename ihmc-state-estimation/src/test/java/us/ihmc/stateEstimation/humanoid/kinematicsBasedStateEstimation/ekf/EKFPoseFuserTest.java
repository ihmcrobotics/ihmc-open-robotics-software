package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.UUID;

import static org.junit.jupiter.api.Assertions.*;

public class EKFPoseFuserTest
{
   private static final long WALL = 1_793_212_000_000_000_000L;
   private static final long MONO = 10_000_000_000L;
   private static final long STEP = 5_000_000L;

   /** A fuser whose output smoother is limited per tick of {@code periodNanoseconds}. */
   private static EKFPoseFuser createFuser(long periodNanoseconds)
   {
      return new EKFPoseFuser(new EKFPoseFusionParameters(), periodNanoseconds * 1.0e-9, new YoRegistry("test"));
   }

   private static RigidBodyTransform pose(double x)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      pose.getTranslation().setX(x);
      return pose;
   }

   /** A pose measurement in the world named {@code frameId}, exposed at {@code exposureTime}. */
   static StampedPosePacket measurement(String frameId, long exposureTime, RigidBodyTransform pose)
   {
      StampedPosePacket packet = new StampedPosePacket();
      packet.getFrameId().append(frameId);
      packet.setTimestamp(exposureTime);
      packet.getPose().set(pose);
      return packet;
   }

   @Test
   public void fusesKinematicsWithoutMutatingInputsAndRetainsOffsetDuringDropoutAndNewWorld()
   {
      EKFPoseFuser fuser = createFuser(STEP);
      String frame = UUID.randomUUID().toString();
      RigidBodyTransform result = new RigidBodyTransform();
      for (int i = 0; i <= 200; i++)
      {
         RigidBodyTransform kinematic = pose(i * .0004);
         fuser.update(WALL + i * STEP, MONO + i * STEP, kinematic, i % 6 == 0 ? measurement(frame, WALL + i * STEP, pose(0)) : null, result);
         assertEquals(i * .0004, kinematic.getTranslationX(), 1e-12);
      }
      assertTrue(result.getTranslationX() < .06, "Pose measurements should reduce the kinematic drift");
      double offset = result.getTranslationX() - .08;
      for (int i = 201; i <= 250; i++)
      {
         fuser.update(WALL + i * STEP, MONO + i * STEP, pose(i * .0004), null, result);
         assertEquals(offset + i * .0004, result.getTranslationX(), 1e-10);
      }
      // The new measurement world is 10 m away: its first measurement realigns to the retained estimate.
      String nextFrame = UUID.randomUUID().toString();
      RigidBodyTransform realigned = new RigidBodyTransform();
      fuser.update(WALL + 251 * STEP, MONO + 251 * STEP, pose(251 * .0004), measurement(nextFrame, WALL + 251 * STEP, pose(10)), realigned);
      assertEquals(result.getTranslationX() + .0004, realigned.getTranslationX(), 1e-10);
   }

   @Test
   public void compensatesDelayedExposureWithKinematicMotionAndRejectsMeasurementsOutsideTheHistory()
   {
      EKFPoseFuser fuser = createFuser(STEP);
      String frame = UUID.randomUUID().toString();
      RigidBodyTransform result = new RigidBodyTransform();
      for (int i = 0; i < 100; i++)
      {
         int exposure = i - 4;
         StampedPosePacket measurement = i >= 4 && i % 6 == 0 ? measurement(frame, WALL + exposure * STEP, pose(exposure * .001)) : null;
         fuser.update(WALL + i * STEP, MONO + i * STEP, pose(i * .001), measurement, result);
         assertEquals(i * .001, result.getTranslationX(), 1e-10);
      }
      fuser.update(WALL + 100 * STEP, MONO + 100 * STEP, pose(.1), measurement(frame, WALL + 101 * STEP, pose(9)), result);
      assertEquals(.1, result.getTranslationX(), 1e-10);
   }

   @Test
   public void oneMeasurementKeepsMovingTheOutputBetweenMeasurementsWithoutRepeatedEkfUpdates()
   {
      EKFPoseFuser fuser = createFuser(1_000_000L);
      String frame = UUID.randomUUID().toString();
      double previous = 0;
      RigidBodyTransform result = new RigidBodyTransform();
      for (int i = 0; i <= 200; i++)
      {
         long exposure = WALL + i * 1_000_000L;
         StampedPosePacket measurement = i == 0 ? measurement(frame, exposure, pose(0)) : i == 33 ? measurement(frame, exposure, pose(.1)) : null;
         fuser.update(exposure, MONO + i * 1_000_000L, pose(0), measurement, result);
         assertTrue(result.getTranslationX() - previous <= .0005 + 1e-12);
         if (i >= 33 && i <= 62)
            assertEquals(.0005, result.getTranslationX() - previous, 1e-10, "The correction must continue on ticks without a measurement");
         if (i == 62)
            assertEquals(.015, result.getTranslationX(), 1e-10);
         previous = result.getTranslationX();
      }
      assertTrue(previous > .05 && previous < .06);
      assertTrue(fuser.getStatus().contains("accepted=1, rejected=0"), fuser.getStatus());
   }

   @Test
   public void millisecondQuantizedWallTimeDoesNotDisableHighRateFusion()
   {
      EKFPoseFuser fuser = createFuser(1_000_000L);
      String frame = UUID.randomUUID().toString();
      RigidBodyTransform result = new RigidBodyTransform();
      fuser.update(WALL, MONO, pose(0), measurement(frame, WALL, pose(0)), result);
      fuser.update(WALL, MONO + 500_000L, pose(.001), null, result);
      fuser.update(WALL + 1_000_000L, MONO + 1_000_000L, pose(.002), measurement(frame, WALL + 1_000_000L, pose(0)), result);
      assertFalse(fuser.getStatus().contains("CLOCK FAULT"));
   }

   @Test
   public void clockDiscontinuityLatchesPredictionOnly()
   {
      EKFPoseFuser fuser = createFuser(STEP);
      String frame = UUID.randomUUID().toString();
      RigidBodyTransform predicted = new RigidBodyTransform();
      fuser.update(WALL, MONO, pose(0), measurement(frame, WALL, pose(0)), predicted);
      fuser.update(WALL + 10 * STEP, MONO + STEP, pose(.01), null, predicted);
      fuser.update(WALL + 11 * STEP, MONO + 2 * STEP, pose(.02), measurement(frame, WALL + 11 * STEP, pose(-.1)), predicted);
      assertEquals(.02, predicted.getTranslationX(), 1e-10);
      assertTrue(fuser.getStatus().contains("CLOCK FAULT"));
   }
}
