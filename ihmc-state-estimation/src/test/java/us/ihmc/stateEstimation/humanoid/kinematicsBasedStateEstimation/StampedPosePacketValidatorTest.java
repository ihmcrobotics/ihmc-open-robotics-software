package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import static org.junit.jupiter.api.Assertions.*;

public class StampedPosePacketValidatorTest
{
   private final SixDoFJoint rootJoint = new SixDoFJoint("pelvis", new RigidBody("elevator", ReferenceFrame.getWorldFrame()));
   private final YoRegistry registry = new YoRegistry("test");
   private final StampedPosePacketValidator validator = new StampedPosePacketValidator(rootJoint, registry);

   @Test
   public void comparesWithTheEstimatedPelvisPoseInterpolatedAtThePacketTimestamp()
   {
      // The estimator's output moves from x = 1.0 to x = 2.0 between two ticks 10 ms apart.
      rootJoint.getJointPose().getPosition().setX(1.0);
      validator.recordEstimatedPelvisPose(1_000_000_000L);
      rootJoint.getJointPose().getPosition().setX(2.0);
      validator.recordEstimatedPelvisPose(1_010_000_000L);

      // Halfway between them, the estimated pose is at x = 1.5.
      assertTrue(validator.isValid(packet(1_005_000_000L, 1.5, 0.0)));
      assertEquals(0.0, registry.findVariable("lastExternalPelvisPositionError").getValueAsDouble(), 1.0e-12);
      assertTrue(validator.isValid(packet(1_005_000_000L, 1.5 + 0.9 * maximumPositionError(), 0.0)));
      assertFalse(validator.isValid(packet(1_005_000_000L, 1.5 + 1.1 * maximumPositionError(), 0.0)));
      assertTrue(validator.isValid(packet(1_005_000_000L, 1.5, 0.9 * maximumOrientationError())));
      assertFalse(validator.isValid(packet(1_005_000_000L, 1.5, 1.1 * maximumOrientationError())));
   }

   @Test
   public void rejectsPacketsOutsideTheRecordedHistoryOrWithNaN()
   {
      validator.recordEstimatedPelvisPose(1_000_000_000L);
      validator.recordEstimatedPelvisPose(1_010_000_000L);

      assertFalse(validator.isValid(packet(999_000_000L, 0.0, 0.0)), "Before the first recorded pose");
      assertFalse(validator.isValid(packet(1_011_000_000L, 0.0, 0.0)), "After the newest recorded pose: nothing is extrapolated");
      assertFalse(validator.isValid(packet(1_005_000_000L, Double.NaN, 0.0)));
      assertTrue(validator.isValid(packet(1_010_000_000L, 0.0, 0.0)));
      assertEquals(1, ((YoLong) registry.findVariable("validExternalPelvisPoseCount")).getValue());
      assertEquals(3, ((YoLong) registry.findVariable("rejectedExternalPelvisPoseCount")).getValue());
   }

   private double maximumPositionError()
   {
      return registry.findVariable("maximumExternalPelvisPositionError").getValueAsDouble();
   }

   private double maximumOrientationError()
   {
      return registry.findVariable("maximumExternalPelvisOrientationError").getValueAsDouble();
   }

   private static StampedPosePacket packet(long timestamp, double x, double yaw)
   {
      StampedPosePacket packet = new StampedPosePacket();
      packet.setTimestamp(timestamp);
      packet.getPose().getPose().getPosition().setX(x);
      packet.getPose().getPose().getOrientation().setToYawOrientation(yaw);
      return packet;
   }
}
