package us.ihmc.perception.vio;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class VIORootPoseCalculatorTest
{
   private final List<Long> convertedExposureTimestamps = new ArrayList<>();
   private final List<RigidBodyTransform> convertedRootPoses = new ArrayList<>();
   private final List<RigidBodyTransform> convertedCameraPoses = new ArrayList<>();
   private final VIORootPoseCalculator calculator = new VIORootPoseCalculator((exposureTimestamp, rootPose, cameraPose) ->
   {
      convertedExposureTimestamps.add(exposureTimestamp);
      convertedRootPoses.add(new RigidBodyTransform(rootPose));
      convertedCameraPoses.add(new RigidBodyTransform(cameraPose));
   });

   private static RigidBodyTransform translation(double x, double y, double z)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(x, y, z);
      return transform;
   }

   @Test
   void interpolatesAtTheExposureAndConvertsTheCameraPoseToARootPoseWithoutReanchoring()
   {
      RigidBodyTransform leftCameraOffset = translation(0.0, 0.06, 0.0);
      calculator.addRobotControllerState(1_000_000_000L, translation(0, 0, 0), translation(0, 0, 1));
      calculator.addCameraTrackingPose(1_010_000_000L, translation(10, 0, 0), leftCameraOffset);
      assertTrue(convertedRootPoses.isEmpty(), "Waits until a robot state after the exposure arrives");
      calculator.addRobotControllerState(1_020_000_000L, translation(0.02, 0, 0), translation(0.02, 0, 1));

      // The controller's pose histories are indexed by wall time.
      assertEquals(1_010_000_000L, convertedExposureTimestamps.get(0));
      assertEquals(0.01, convertedRootPoses.get(0).getTranslationX(), 1.0e-10);
      assertEquals(0.0, convertedRootPoses.get(0).getTranslationY(), 1.0e-10);
      assertEquals(0.0, convertedRootPoses.get(0).getTranslationZ(), 1.0e-10);
      assertEquals(0.01, convertedCameraPoses.get(0).getTranslationX(), 1.0e-10);
      assertEquals(0.06, convertedCameraPoses.get(0).getTranslationY(), 1.0e-10);
      assertEquals(1.0, convertedCameraPoses.get(0).getTranslationZ(), 1.0e-10);

      // KIO moves 2 cm while VIO moves 3 cm: the anchor from the first pose is kept.
      calculator.addCameraTrackingPose(1_030_000_000L, translation(10.03, 0, 0), leftCameraOffset);
      calculator.addRobotControllerState(1_040_000_000L, translation(0.04, 0, 0), translation(0.04, 0, 1));
      assertEquals(0.04, convertedRootPoses.get(1).getTranslationX(), 1.0e-10);
      assertEquals(0.04, convertedCameraPoses.get(1).getTranslationX(), 1.0e-10);
      assertEquals(0.06, convertedCameraPoses.get(1).getTranslationY(), 1.0e-10);
      assertEquals(1.0, convertedCameraPoses.get(1).getTranslationZ(), 1.0e-10);
      assertEquals(2, calculator.getAcceptedCount());
   }

   @Test
   void rejectsLostTrackingAndConvertsEveryOtherPoseHoweverFarFromTheRobotState()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      calculator.addRobotControllerState(1_000_000_000L, identity, identity);
      calculator.addRobotControllerState(1_020_000_000L, identity, identity);
      calculator.addCameraTrackingPose(1_010_000_000L, identity, identity);
      calculator.addCameraTrackingPose(1_011_000_000L, null, identity);
      calculator.addCameraTrackingPose(1_012_000_000L, translation(1, 0, 0), identity);
      RigidBodyTransform rotatedTwentyDegrees = new RigidBodyTransform();
      rotatedTwentyDegrees.getRotation().setToYawOrientation(Math.toRadians(20));
      calculator.addCameraTrackingPose(1_013_000_000L, rotatedTwentyDegrees, identity);

      // Whether a pose is too far from the robot state is decided in the controller process.
      assertEquals(3, convertedRootPoses.size());
      assertEquals(3, calculator.getAcceptedCount());
      assertEquals(1, calculator.getRejectedCount());
      assertTrue(convertedCameraPoses.get(0).epsilonEquals(identity, 1.0e-12));
      assertEquals(1.0, convertedRootPoses.get(1).getTranslationX(), 1.0e-10);
   }

   @Test
   void lateRobotStatesAreIgnored()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      calculator.addRobotControllerState(1_000_000_000L, translation(0, 0, 0), translation(0, 0, 0));
      calculator.addRobotControllerState(1_040_000_000L, translation(0.04, 0, 0), translation(0.04, 0, 0));
      calculator.addCameraTrackingPose(1_030_000_000L, translation(5, 0, 0), identity); // Anchors against the robot states at 1.00 and 1.04 s.
      calculator.addCameraTrackingPose(1_050_000_000L, translation(5.02, 0, 0), identity);
      // Arrives late and is off the line, so it would change the result if it were used to interpolate at the second exposure.
      calculator.addRobotControllerState(1_020_000_000L, translation(0.5, 0, 0), translation(0.5, 0, 0));
      calculator.addRobotControllerState(1_060_000_000L, translation(0.06, 0, 0), translation(0.06, 0, 0));

      assertEquals(2, convertedExposureTimestamps.size());
      assertEquals(1_050_000_000L, convertedExposureTimestamps.get(1));
      assertEquals(0.05, convertedRootPoses.get(1).getTranslationX(), 1.0e-10);
   }

   @Test
   void anExposureAtTheNewestStateWaitsAndRobotStatesTooFarApartAreRejected()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      calculator.addRobotControllerState(1_000_000_000L, identity, identity);
      calculator.addRobotControllerState(1_020_000_000L, identity, identity);
      calculator.addCameraTrackingPose(1_020_000_000L, identity, identity);
      assertTrue(convertedExposureTimestamps.isEmpty());
      calculator.addRobotControllerState(1_040_000_000L, identity, identity);
      assertEquals(1_020_000_000L, convertedExposureTimestamps.get(0));

      calculator.addCameraTrackingPose(1_050_000_000L, identity, identity);
      calculator.addRobotControllerState(1_110_000_000L, identity, identity); // 70 ms after the previous state: too wide to interpolate.
      assertEquals(1, convertedExposureTimestamps.size());
   }

   @Test
   void resetDiscardsTheHistoryPendingPosesAndAnchorWithoutMovingTheKIOWorld()
   {
      RigidBodyTransform identity = new RigidBodyTransform();
      calculator.addRobotControllerState(1_000_000_000L, translation(1, 2, 1), translation(1, 2, 2));
      calculator.addRobotControllerState(1_020_000_000L, translation(1, 2, 1), translation(1, 2, 2));
      calculator.addCameraTrackingPose(1_010_000_000L, translation(10, 0, 0), identity);
      assertTrue(calculator.areWorldFramesAnchoredToOneAnother());
      calculator.addCameraTrackingPose(1_030_000_000L, translation(10, 0, 0), identity); // Pending: must not survive the reset.

      calculator.resetForNewSession();
      assertFalse(calculator.areWorldFramesAnchoredToOneAnother());
      assertEquals(0, calculator.getAcceptedCount());
      calculator.addCameraTrackingPose(2_010_000_000L, translation(-50, 0, 0), identity);
      assertEquals(1, convertedRootPoses.size(), "The old session's robot states can't be used to interpolate in a new session");
      calculator.addRobotControllerState(2_000_000_000L, translation(3, 4, 1.2), translation(3, 4, 2.2));
      calculator.addRobotControllerState(2_020_000_000L, translation(3, 4, 1.2), translation(3, 4, 2.2));
      assertEquals(2, convertedRootPoses.size());
      assertTrue(convertedRootPoses.get(1).epsilonEquals(translation(3, 4, 1.2), 1.0e-10));
   }

   @Test
   void convertsThroughARotatedCameraMounting()
   {
      RigidBodyTransform vioWorldToRobotWorld = translation(2, 3, 0);
      vioWorldToRobotWorld.getRotation().setToYawOrientation(0.8);
      RigidBodyTransform trackedCameraPose = translation(1, 0, 1);
      trackedCameraPose.getRotation().setToPitchOrientation(0.2);
      RigidBodyTransform robotStateCameraPose = new RigidBodyTransform(vioWorldToRobotWorld);
      robotStateCameraPose.multiply(trackedCameraPose);
      RigidBodyTransform robotStateRootPose = translation(2, 4, 0);
      robotStateRootPose.getRotation().setToYawOrientation(0.3);

      RigidBodyTransform rootPose = VIORootPoseCalculator.computeRootPoseFromTrackedCameraPose(vioWorldToRobotWorld,
                                                                                                   trackedCameraPose,
                                                                                                   robotStateCameraPose,
                                                                                                   robotStateRootPose);
      assertTrue(rootPose.epsilonEquals(robotStateRootPose, 1.0e-10));
   }
}
