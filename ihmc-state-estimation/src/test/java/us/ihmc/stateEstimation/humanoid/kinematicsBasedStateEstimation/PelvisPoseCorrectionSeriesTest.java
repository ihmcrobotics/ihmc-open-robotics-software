package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

public class PelvisPoseCorrectionSeriesTest
{
   private final List<String> runOrder = new ArrayList<>();
   private final ExternalPoses externalPoses = new ExternalPoses();
   /** Stays at the identity, like the poses offered, so every offered pose whose timestamp was recorded is valid. */
   private final SixDoFJoint rootJoint = new SixDoFJoint("pelvis", new RigidBody("elevator", ReferenceFrame.getWorldFrame()));

   /** A series whose validator has already recorded the estimator's pelvis pose at timestamps 1 to 4. */
   private PelvisPoseCorrectionSeries createSeriesWithRecordedPelvisPoses()
   {
      PelvisPoseCorrectionSeries series = new PelvisPoseCorrectionSeries(new StampedPosePacketValidator(rootJoint, new YoRegistry("test")));
      for (long timestamp = 1; timestamp <= 4; timestamp++)
         series.doControl(timestamp);
      return series;
   }

   @Test
   public void runsCorrectionsInOrderAndOffersEachNewPoseToEveryOne()
   {
      RecordingCorrection first = new RecordingCorrection("first");
      RecordingCorrection second = new RecordingCorrection("second");
      PelvisPoseCorrectionSeries series = createSeriesWithRecordedPelvisPoses();
      series.addCorrection(first);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);
      series.addCorrection(second); // Added after the subscriber: still gets its own view.

      externalPoses.offer(2);
      series.doControl(5);
      assertEquals(List.of("first", "second"), runOrder);
      assertEquals(List.of(2L), first.receivedTimestamps);
      assertEquals(List.of(2L), second.receivedTimestamps);

      series.doControl(6);
      assertEquals(List.of(2L), first.receivedTimestamps, "A pose is offered once");
      assertEquals(List.of(2L), second.receivedTimestamps);
   }

   @Test
   public void skipsInactiveCorrectionsAndDropsThePosesTheyMissed()
   {
      RecordingCorrection first = new RecordingCorrection("first");
      RecordingCorrection second = new RecordingCorrection("second");
      second.setActive(false);
      PelvisPoseCorrectionSeries series = createSeriesWithRecordedPelvisPoses();
      series.addCorrection(first);
      series.addCorrection(second);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);

      externalPoses.offer(2);
      series.doControl(5);
      assertEquals(List.of("first"), runOrder);
      assertEquals(List.of(2L), first.receivedTimestamps);

      second.setActive(true);
      series.doControl(6);
      assertEquals(List.of("first", "first", "second"), runOrder);
      assertTrue(second.receivedTimestamps.isEmpty(), "A pose that arrived while inactive is not delivered later");

      externalPoses.offer(3);
      series.doControl(7);
      assertEquals(List.of(3L), second.receivedTimestamps);
   }

   @Test
   public void modesSwitchTheTwoCorrectionsAndCoverEveryCombination()
   {
      RecordingCorrection nativeCorrection = new RecordingCorrection("native");
      RecordingCorrection ekfCorrection = new RecordingCorrection("ekf");
      PelvisPoseCorrectionSeries series = createSeriesWithRecordedPelvisPoses();
      series.addCorrection(nativeCorrection);
      series.addCorrection(ekfCorrection);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);

      Map<PelvisPoseCorrectionMode, List<String>> expectedRunOrders = Map.of(PelvisPoseCorrectionMode.NONE,
                                List.<String>of(),
                                PelvisPoseCorrectionMode.NATIVE,
                                List.of("native"),
                                PelvisPoseCorrectionMode.EKF,
                                List.of("ekf"),
                                PelvisPoseCorrectionMode.NATIVE_THEN_EKF,
                                List.of("native", "ekf"));
      assertEquals(PelvisPoseCorrectionMode.values().length, expectedRunOrders.size());
      for (PelvisPoseCorrectionMode mode : PelvisPoseCorrectionMode.values())
      {
         mode.apply(nativeCorrection, ekfCorrection);
         runOrder.clear();
         series.doControl(5);
         assertEquals(expectedRunOrders.get(mode), runOrder, mode.name());
         assertEquals(mode.usesExternalPoses(), series.isActive());
      }

      series.setActive(false);
      assertFalse(nativeCorrection.isActive());
      assertFalse(ekfCorrection.isActive());
   }

   @Test
   public void forwardsOutgoingPacketsToTheRealSubscriber()
   {
      RecordingCorrection correction = new RecordingCorrection("only");
      PelvisPoseCorrectionSeries series = createSeriesWithRecordedPelvisPoses();
      series.addCorrection(correction);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);
      correction.subscriber.sendPelvisPoseErrorPacket(new PelvisPoseErrorPacket());
      correction.subscriber.sendLocalizationResetRequest(new LocalizationPacket());
      assertEquals(1, externalPoses.errorPackets);
      assertEquals(1, externalPoses.resetRequests);
   }

   @Test
   public void posesTheValidatorRejectsReachNoCorrection()
   {
      RecordingCorrection correction = new RecordingCorrection("only");
      PelvisPoseCorrectionSeries series = createSeriesWithRecordedPelvisPoses();
      series.addCorrection(correction);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);

      externalPoses.offer(10); // After every recorded pelvis pose, so it can't be compared.
      series.doControl(5);
      assertTrue(correction.receivedTimestamps.isEmpty());

      externalPoses.offer(2);
      externalPoses.packet.getPose().getPose().getPosition().setX(1.0); // Further from the estimator's pelvis pose than the limit.
      series.doControl(6);
      assertTrue(correction.receivedTimestamps.isEmpty());

      externalPoses.packet.getPose().getPose().getPosition().setX(0.0);
      externalPoses.offer(3);
      series.doControl(7);
      assertEquals(List.of(3L), correction.receivedTimestamps);
   }

   private class RecordingCorrection implements PelvisPoseHistoryCorrectionInterface
   {
      private final String name;
      private final List<Long> receivedTimestamps = new ArrayList<>();
      private PelvisPoseCorrectionCommunicatorInterface subscriber;
      private boolean active = true;

      private RecordingCorrection(String name)
      {
         this.name = name;
      }

      @Override
      public void doControl(long timestamp)
      {
         runOrder.add(name);
         if (subscriber.hasNewPose())
            receivedTimestamps.add(subscriber.getNewExternalPose().getTimestamp());
      }

      @Override
      public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
      {
         subscriber = externalPelvisPoseSubscriber;
      }

      @Override
      public boolean isActive()
      {
         return active;
      }

      @Override
      public void setActive(boolean active)
      {
         this.active = active;
      }

      @Override
      public YoGraphicDefinition getSCS2YoGraphics()
      {
         return null;
      }
   }

   private static class ExternalPoses implements PelvisPoseCorrectionCommunicatorInterface
   {
      private final StampedPosePacket packet = new StampedPosePacket();
      private boolean hasNewPose;
      private int errorPackets, resetRequests;

      void offer(long timestamp)
      {
         packet.setTimestamp(timestamp);
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
         errorPackets++;
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         resetRequests++;
      }

      @Override
      public void receivedPacket(StampedPosePacket packet)
      {
      }
   }
}
