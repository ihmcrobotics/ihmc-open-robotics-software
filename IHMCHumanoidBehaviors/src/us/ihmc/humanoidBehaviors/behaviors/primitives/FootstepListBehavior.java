package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;

public class FootstepListBehavior extends BehaviorInterface
{
   private static final boolean DEBUG = false;

   private FootstepDataList outgoingFootstepDataList;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue;
   private FootstepStatus lastFootstepStatus;

   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private final IntegerYoVariable numberOfFootsteps = new IntegerYoVariable("numberOfFootsteps" + behaviorName, registry);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final BooleanYoVariable isStopped = new BooleanYoVariable("isStopped", registry);
   
   public FootstepListBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      attachControllerListeningQueue(footstepStatusQueue, FootstepStatus.class);
      numberOfFootsteps.set(-1);
   }

   public void set(FootstepDataList footStepList)
   {
      outgoingFootstepDataList = footStepList;
      numberOfFootsteps.set(outgoingFootstepDataList.getDataList().size());
   }

   @Override
   public void doControl()
   {
      checkForNewFootstepStatusPacket();

      if (!packetHasBeenSent.getBooleanValue() && outgoingFootstepDataList != null)
      {
         sendFootsepListToController();
      }
   }

   private void sendFootsepListToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         sendPacketToController(outgoingFootstepDataList);
         packetHasBeenSent.set(true);
      }
   }

   private void checkForNewFootstepStatusPacket()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         lastFootstepStatus = footstepStatusQueue.getNewestPacket();
      }
   }

   @Override
   public void initialize()
   {
	   numberOfFootsteps.set(-1);
   }

   @Override
   public void finalize()
   {
      footstepStatusQueue.clear();
      outgoingFootstepDataList = null;
      packetHasBeenSent.set(false);
      numberOfFootsteps.set(-1);

      lastFootstepStatus = null;
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public void stop()
   {
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      if (lastFootstepStatus == null)
         return false;

      if (DEBUG)
         System.out.println("FootstepStatus isn't null");

      boolean isLastFootstep = lastFootstepStatus.getFootstepIndex() >= numberOfFootsteps.getIntegerValue() - 1;
      if (DEBUG)
         System.out.println("isLastFootstep returning: " + isLastFootstep + ", total nb of footsteps: " + numberOfFootsteps + ", current footstep: "
               + lastFootstepStatus.getFootstepIndex());

      boolean isCompleted = lastFootstepStatus.getStatus() == Status.COMPLETED;
      if (DEBUG)
         System.out.println("isCompleted returning: " + isCompleted);

      boolean isDone = isLastFootstep && isCompleted && !isPaused.getBooleanValue();
      if (DEBUG)
         System.out.println("isDone() returning: " + isDone);

      return isDone;
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }
}
