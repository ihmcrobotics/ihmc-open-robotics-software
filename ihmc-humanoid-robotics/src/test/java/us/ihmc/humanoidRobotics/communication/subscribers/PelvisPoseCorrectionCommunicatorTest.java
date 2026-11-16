package us.ihmc.humanoidRobotics.communication.subscribers;

import ihmc_common_msgs.StampedPosePacket;
import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.*;

public class PelvisPoseCorrectionCommunicatorTest
{
   @Test
   public void copiesAReusedPacketAndHandsPosesOverInOrder()
   {
      var communicator = new PelvisPoseCorrectionCommunicator(null, null);
      var reused = new StampedPosePacket(); // Like a subscription sampler's sample.
      for (long timestamp = 1; timestamp <= 3; timestamp++)
      {
         reused.setTimestamp(timestamp);
         communicator.receivedPacket(reused);
      }
      reused.setTimestamp(99);

      for (long expected = 1; expected <= 3; expected++)
      {
         assertTrue(communicator.hasNewPose());
         StampedPosePacket pose = communicator.getNewExternalPose();
         assertNotSame(reused, pose);
         assertEquals(expected, pose.getTimestamp());
      }
      assertFalse(communicator.hasNewPose());
      assertNull(communicator.getNewExternalPose());
   }

   @Test
   public void returnedPoseStaysValidUntilTheNextOneIsTaken()
   {
      var communicator = new PelvisPoseCorrectionCommunicator(null, null);
      var packet = new StampedPosePacket();
      packet.setTimestamp(1);
      communicator.receivedPacket(packet);
      StampedPosePacket first = communicator.getNewExternalPose();

      for (long timestamp = 2; timestamp < 100; timestamp++)
      {
         packet.setTimestamp(timestamp);
         communicator.receivedPacket(packet);
      }
      assertEquals(1, first.getTimestamp());
   }

   @Test
   public void overwritesTheOldestPosesWhenFull()
   {
      var communicator = new PelvisPoseCorrectionCommunicator(null, null);
      var packet = new StampedPosePacket();
      for (long timestamp = 1; timestamp <= 100; timestamp++)
      {
         packet.setTimestamp(timestamp);
         communicator.receivedPacket(packet);
      }

      long first = communicator.getNewExternalPose().getTimestamp();
      long last = first;
      while (communicator.hasNewPose())
      {
         long timestamp = communicator.getNewExternalPose().getTimestamp();
         assertEquals(last + 1, timestamp, "Poses come out in order, oldest first");
         last = timestamp;
      }
      assertEquals(100, last, "The newest pose is always kept");
      assertTrue(first > 80, "The oldest poses were overwritten, so only the most recent ones remain: first = " + first);

      packet.setTimestamp(200);
      communicator.receivedPacket(packet);
      assertEquals(200, communicator.getNewExternalPose().getTimestamp());
   }

   @Test
   public void concurrentReadsAreConsistentAndInOrder() throws InterruptedException
   {
      var communicator = new PelvisPoseCorrectionCommunicator(null, null);
      var done = new AtomicBoolean();
      var writer = new Thread(() ->
      {
         var packet = new StampedPosePacket(); // Reused, like a subscription sampler's sample.
         for (long timestamp = 1; timestamp <= 2_000_000; timestamp++)
         {
            packet.setTimestamp(timestamp);
            packet.getPose().getPose().getPosition().setX(timestamp);
            packet.getFrameId().setLength(0);
            packet.getFrameId().append(timestamp % 3 == 0 ? "a" : "session-" + timestamp);
            communicator.receivedPacket(packet);
         }
         done.set(true);
      });
      writer.start();

      long last = 0;
      long taken = 0;
      while (!done.get() || communicator.hasNewPose())
      {
         StampedPosePacket pose = communicator.getNewExternalPose();
         if (pose == null)
            continue;
         long timestamp = pose.getTimestamp();
         assertTrue(timestamp > last, "Out of order or repeated: " + timestamp + " after " + last);
         assertEquals(timestamp, pose.getPose().getPose().getPosition().getX(), "Torn read at " + timestamp);
         assertEquals(timestamp % 3 == 0 ? "a" : "session-" + timestamp, pose.getFrameIdAsString(), "Torn read at " + timestamp);
         last = timestamp;
         taken++;
      }
      writer.join();
      assertEquals(2_000_000, last, "The last pose written is always delivered");
      assertTrue(taken > 0);
   }
}
