package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class PelvisPoseCorrectionSeriesTest
{
   private final List<String> runOrder = new ArrayList<>();
   private final ExternalPoses externalPoses = new ExternalPoses();

   @Test
   public void runsCorrectionsInOrderAndOffersEachNewPoseToEveryOne()
   {
      var first = new RecordingCorrection("first");
      var second = new RecordingCorrection("second");
      var series = new PelvisPoseCorrectionSeries();
      series.addCorrection(first);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);
      series.addCorrection(second); // Added after the subscriber: still gets its own view.

      externalPoses.offer(5);
      series.doControl(1);
      assertEquals(List.of("first", "second"), runOrder);
      assertEquals(List.of(5L), first.receivedTimestamps);
      assertEquals(List.of(5L), second.receivedTimestamps);

      series.doControl(2);
      assertEquals(List.of(5L), first.receivedTimestamps, "A pose is offered once");
      assertEquals(List.of(5L), second.receivedTimestamps);
   }

   @Test
   public void skipsInactiveCorrectionsAndDropsThePosesTheyMissed()
   {
      var first = new RecordingCorrection("first");
      var second = new RecordingCorrection("second");
      var secondActive = new AtomicBoolean(false);
      var series = new PelvisPoseCorrectionSeries();
      series.addCorrection(first);
      series.addCorrection(second, secondActive::get);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);

      externalPoses.offer(5);
      series.doControl(1);
      assertEquals(List.of("first"), runOrder);
      assertEquals(List.of(5L), first.receivedTimestamps);

      secondActive.set(true);
      series.doControl(2);
      assertEquals(List.of("first", "first", "second"), runOrder);
      assertTrue(second.receivedTimestamps.isEmpty(), "A pose that arrived while inactive is not delivered later");

      externalPoses.offer(7);
      series.doControl(3);
      assertEquals(List.of(7L), second.receivedTimestamps);
   }

   @Test
   public void forwardsOutgoingPacketsToTheRealSubscriber()
   {
      var correction = new RecordingCorrection("only");
      var series = new PelvisPoseCorrectionSeries();
      series.addCorrection(correction);
      series.setExternalPelvisCorrectorSubscriber(externalPoses);
      correction.subscriber.sendPelvisPoseErrorPacket(new PelvisPoseErrorPacket());
      correction.subscriber.sendLocalizationResetRequest(new LocalizationPacket());
      assertEquals(1, externalPoses.errorPackets);
      assertEquals(1, externalPoses.resetRequests);
   }

   private class RecordingCorrection implements PelvisPoseHistoryCorrectionInterface
   {
      private final String name;
      private final List<Long> receivedTimestamps = new ArrayList<>();
      private PelvisPoseCorrectionCommunicatorInterface subscriber;

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
