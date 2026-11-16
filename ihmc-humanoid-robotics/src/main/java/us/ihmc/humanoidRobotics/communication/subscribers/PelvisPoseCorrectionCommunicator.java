package us.ihmc.humanoidRobotics.communication.subscribers;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.StampedLock;

/**
 * Passes external pelvis poses to the state estimator's pelvis pose correction, and publishes the correction's pose error and
 * localization reset packets.
 * <p>
 * Given a node, it subscribes to the robot's standard external pose topic,
 * {@code StateEstimatorAPI.getTopic(StampedPosePacket.class, robotName)}, and publishes on the matching standard topics. Use an
 * {@code AsyncROS2Node}, so publishing from the estimator thread never blocks or allocates.
 * </p>
 * <p>
 * {@link #receivedPacket(StampedPosePacket)} copies each pose into a fixed-size, preallocated ring buffer, so the caller may
 * reuse its packet, for example the sample of a subscription sampler. When the buffer is full, each new pose overwrites the
 * oldest one, so the newest poses are kept. It must be called from one thread at a time.
 * </p>
 * <p>
 * The correction takes the poses oldest first, one per call to {@link #getNewExternalPose()}, without allocating, locking or
 * waiting: it copies a slot optimistically and retries if the network thread wrote to it meanwhile. If the network thread has
 * overwritten poses the correction had not taken yet, those are skipped.
 * </p>
 */
public class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
{
   private static final int POSE_BUFFER_CAPACITY = 16;

   private final Slot[] slots = new Slot[POSE_BUFFER_CAPACITY];
   /** Number of poses written so far. Written only by the network thread. */
   private final AtomicLong writeCount = new AtomicLong();
   /** Index of the next pose to take. Used only by the estimator thread. */
   private long readIndex;
   /** The estimator thread's copy of the pose it took last. */
   private final StampedPosePacket takenPose = new StampedPosePacket();

   private final ROS2Publisher<PelvisPoseErrorPacket> poseErrorPublisher;
   private final ROS2Publisher<LocalizationPacket> localizationPublisher;

   private static class Slot
   {
      private final StampedLock lock = new StampedLock();
      private final StampedPosePacket pose = new StampedPosePacket();
      /** Which pose, counting from 0, this slot holds. */
      private long index = -1;
   }

   /** With a {@code null} node, nothing is subscribed or published, and poses are fed through {@link #receivedPacket}. */
   public PelvisPoseCorrectionCommunicator(ROS2Node asyncROS2Node, String robotName)
   {
      for (int i = 0; i < slots.length; i++)
         slots[i] = new Slot();

      if (asyncROS2Node != null && robotName != null)
      {
         poseErrorPublisher = asyncROS2Node.createPublisher(StateEstimatorAPI.getTopic(PelvisPoseErrorPacket.class, robotName));
         localizationPublisher = asyncROS2Node.createPublisher(StateEstimatorAPI.getTopic(LocalizationPacket.class, robotName));
         // The sampler reuses one packet; receivedPacket copies it.
         asyncROS2Node.createSubscriptionSampler(StateEstimatorAPI.getTopic(StampedPosePacket.class, robotName), this::receivedPacket);
      }
      else
      {
         poseErrorPublisher = null;
         localizationPublisher = null;
      }
   }

   /** On the network thread: copies the packet into the buffer, overwriting the oldest pose if it is full. */
   @Override
   public void receivedPacket(StampedPosePacket newestStampedPosePacket)
   {
      long index = writeCount.get();
      Slot slot = slots[slotIndex(index)];
      // Only this thread takes the write lock, and readers never lock, so this never waits.
      long stamp = slot.lock.writeLock();
      try
      {
         slot.pose.set(newestStampedPosePacket);
         slot.index = index;
      }
      finally
      {
         slot.lock.unlockWrite(stamp);
      }
      writeCount.set(index + 1);
   }

   @Override
   public boolean hasNewPose()
   {
      return writeCount.get() > readIndex;
   }

   /**
    * The oldest pose not yet taken, or {@code null}. The returned packet is owned by this communicator and stays valid until the
    * next call.
    */
   @Override
   public StampedPosePacket getNewExternalPose()
   {
      // Each failed attempt means the network thread moved on, so a few attempts always reach a stable slot in practice.
      for (int attempt = 0; attempt < POSE_BUFFER_CAPACITY; attempt++)
      {
         long written = writeCount.get();
         if (readIndex >= written)
            return null;
         // Skip poses that have been overwritten, and the oldest one too: it is the next to be overwritten, so reading it would race
         // the network thread.
         readIndex = Math.max(readIndex, written - POSE_BUFFER_CAPACITY + 1);

         Slot slot = slots[slotIndex(readIndex)];
         long stamp = slot.lock.tryOptimisticRead();
         if (stamp != 0 && copyOptimistically(slot) && slot.lock.validate(stamp) && slot.index == readIndex)
         {
            readIndex++;
            return takenPose;
         }
         // The slot was being written, or already holds a newer pose: look again from the oldest pose still in the buffer.
         readIndex++;
      }
      return null;
   }

   /**
    * Copies the slot without locking. The copy may be inconsistent if the slot is written meanwhile, which the caller detects with
    * {@link StampedLock#validate(long)}; copying a string while it changes can also throw, which counts as a failed copy.
    */
   private boolean copyOptimistically(Slot slot)
   {
      try
      {
         takenPose.set(slot.pose);
         return true;
      }
      catch (RuntimeException inconsistentRead)
      {
         return false;
      }
   }

   private static int slotIndex(long index)
   {
      return (int) (index % POSE_BUFFER_CAPACITY);
   }

   @Override
   public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
   {
      if (poseErrorPublisher != null)
         poseErrorPublisher.publish(pelvisPoseErrorPacket);
   }

   @Override
   public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
   {
      if (localizationPublisher != null)
         localizationPublisher.publish(localizationPacket);
   }
}
