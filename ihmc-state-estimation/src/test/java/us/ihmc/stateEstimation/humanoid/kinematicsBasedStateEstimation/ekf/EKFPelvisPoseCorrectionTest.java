package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

public class EKFPelvisPoseCorrectionTest
{
   private static final long WALL = 1_793_212_000_000_000_000L, MONO = 10_000_000_000L, TICK = 1_000_000L;

   private final AtomicLong monotonicTime = new AtomicLong(MONO);
   private final SixDoFJoint rootJoint = new SixDoFJoint("pelvis", new RigidBody("elevator", ReferenceFrame.getWorldFrame()));
   private final YoRegistry registry = new YoRegistry("test");
   private final EKFPelvisPoseCorrection correction = new EKFPelvisPoseCorrection(rootJoint, monotonicTime::get, new FusedPoseEstimatorParameters(), registry);
   private final PoseQueue measurements = new PoseQueue();
   private final List<StampedPosePacket> published = new ArrayList<>();
   private final RigidBodyTransform kinematicPose = new RigidBodyTransform();
   private final RigidBodyTransform rootJointPose = new RigidBodyTransform();

   public EKFPelvisPoseCorrectionTest()
   {
      new RigidBody("pelvisBody", rootJoint, 0.1, 0.1, 0.1, 10.0, new Vector3D());
      correction.setCorrectedPosePublisher(packet ->
                                           {
                                              var copy = new StampedPosePacket();
                                              copy.set(packet);
                                              published.add(copy);
                                           });
   }

   @Test
   public void appliesTheCorrectionToThePelvisAndPublishesIt()
   {
      correction.setExternalPelvisCorrectorSubscriber(measurements);
      runTicks(0, 200);

      rootJoint.getJointConfiguration(rootJointPose);
      assertTrue(rootJointPose.getTranslationX() > .05, "The pelvis should move toward the measured pose: " + rootJointPose.getTranslationX());
      assertTrue(rootJointPose.getTranslationX() <= .1 + 1e-9);

      assertEquals(10, published.size(), "One corrected pose per 20 ms publish period");
      for (int i = 0; i < published.size(); i++)
      {
         assertEquals(i + 1, published.get(i).getSequenceId());
         assertEquals(WALL + i * 20 * TICK, published.get(i).getTimestamp());
         assertEquals(ReferenceFrame.getWorldFrame().getName(), published.get(i).getFrameIdAsString());
      }
      assertTrue(published.get(published.size() - 1).getPose().getPose().getX() > .04);
   }

   @Test
   public void publishOnlyLeavesThePelvisUntilTheYoVariableSwitchesToApply()
   {
      correction.setExternalPelvisCorrectorSubscriber(measurements);
      YoBoolean apply = (YoBoolean) registry.findVariable("applyEKFPelvisPoseCorrection");
      apply.set(false);
      runTicks(0, 200);

      rootJoint.getJointConfiguration(rootJointPose);
      assertEquals(0.0, rootJointPose.getTranslationX(), 0.0, "Publish-only must not change the pelvis");
      double publishedX = published.get(published.size() - 1).getPose().getPose().getX();
      assertTrue(publishedX > .04, "The corrected pose is still computed and published: " + publishedX);

      apply.set(true);
      runTicks(200, 201);
      rootJoint.getJointConfiguration(rootJointPose);
      assertTrue(rootJointPose.getTranslationX() > .05, "Switching to apply uses the correction kept while publishing only");
   }

   @Test
   public void doesNothingWithoutAPoseMeasurementSubscriber()
   {
      runTicks(0, 50);
      rootJoint.getJointConfiguration(rootJointPose);
      assertTrue(rootJointPose.epsilonEquals(kinematicPose, 0.0));
      assertTrue(published.isEmpty());
   }

   /**
    * Like the estimator, sets the uncorrected pelvis pose at the origin before each correction. The first measurement aligns the
    * measurement world; the rest put the pelvis 0.1 m forward.
    */
   private void runTicks(int first, int last)
   {
      for (int i = first; i < last; i++)
      {
         long wallTime = WALL + i * TICK;
         monotonicTime.set(MONO + i * TICK);
         rootJoint.setJointConfiguration(kinematicPose);
         rootJoint.updateFrame();
         var measuredPose = new RigidBodyTransform();
         if (i > 0)
            measuredPose.getTranslation().setX(.1);
         measurements.offer("measurementWorld", wallTime, measuredPose);
         correction.doControl(wallTime);
      }
   }

   private static class PoseQueue implements PelvisPoseCorrectionCommunicatorInterface
   {
      private final StampedPosePacket packet = new StampedPosePacket();
      private boolean hasNewPose;

      void offer(String frameId, long timestamp, RigidBodyTransform pose)
      {
         packet.getFrameId().setLength(0);
         packet.getFrameId().append(frameId);
         packet.setTimestamp(timestamp);
         packet.getPose().set(pose);
         hasNewPose = true;
      }

      @Override
      public boolean hasNewPose()
      {
         return hasNewPose;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         hasNewPose = false;
         return packet;
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
      }

      @Override
      public void receivedPacket(StampedPosePacket packet)
      {
      }
   }
}
