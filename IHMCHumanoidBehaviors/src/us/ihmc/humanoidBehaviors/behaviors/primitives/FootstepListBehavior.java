package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.PauseCommand;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

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
   private final BooleanYoVariable hasLastStepBeenReached = new BooleanYoVariable("hasLastStepBeenReached", registry);
   private boolean isInitialized;

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
      packetHasBeenSent.set(false);
   }

   public void set(ArrayList<Footstep> footsteps)
   {
      FootstepDataList footsepDataList = new FootstepDataList();

      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Point3d location = new Point3d(footstep.getX(), footstep.getY(), footstep.getZ());
         Quat4d orientation = new Quat4d();
         footstep.getOrientation(orientation);

         RobotSide footstepSide = footstep.getRobotSide();
         FootstepData footstepData = new FootstepData(footstepSide, location, orientation);
         footsepDataList.add(footstepData);
      }
      set(footsepDataList);
   }

   @Override
   public void doControl()
   {
      checkForNewFootstepStatusPacket();

      if (!packetHasBeenSent.getBooleanValue() && outgoingFootstepDataList != null)
      {
         sendFootsepListToController();
         //Send to Network processor for downlink to UI
      }
   }

   private void sendFootsepListToController()
   {
      if (!isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         outgoingFootstepDataList.setDestination(PacketDestination.UI);

         sendPacketToNetworkProcessor(outgoingFootstepDataList);
         sendPacketToController(outgoingFootstepDataList);
         packetHasBeenSent.set(true);
      }
   }

   private void checkForNewFootstepStatusPacket()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         lastFootstepStatus = footstepStatusQueue.getNewestPacket();
         
         boolean isLastFootstep = lastFootstepStatus.getFootstepIndex() >= numberOfFootsteps.getIntegerValue() - 1;

         if (isLastFootstep)
         {
            hasLastStepBeenReached.set(true);
            SysoutTool.println("FootstepListBehavior is Done.", DEBUG);
         }

         SysoutTool.println("isLastFootstep: " + isLastFootstep + ", total nb of footsteps: " + numberOfFootsteps + ", current footstep: "
               + lastFootstepStatus.getFootstepIndex(), DEBUG);
      }
   }

   @Override
   public void initialize()
   {
      if (!isInitialized)
      {
         packetHasBeenSent.set(false);
         hasLastStepBeenReached.set(false);

         isPaused.set(false);
         isStopped.set(false);
         isInitialized = true;

      }
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
      isInitialized = false;
      hasLastStepBeenReached.set(false);
   }

   @Override
   public void stop()
   {
      sendPacketToController(new PauseCommand(true));
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      sendPacketToController(new PauseCommand(true));
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      sendPacketToController(new PauseCommand(false));
      isPaused.set(false);
      isStopped.set(false);
   }

   @Override
   public boolean isDone()
   {
      if (lastFootstepStatus == null)
         return false;

      boolean isDone = lastFootstepStatus.isDoneWalking() && hasLastStepBeenReached.getBooleanValue() && !isPaused.getBooleanValue();

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

   @Override
   public boolean hasInputBeenSet()
   {
      boolean receivedFootStepStatusReplyFromController = lastFootstepStatus != null;
      if (numberOfFootsteps.getIntegerValue() != -1 && receivedFootStepStatusReplyFromController)
         return true;
      else
         return false;
   }

   public boolean isWalking()
   {
      return hasInputBeenSet() && !isDone();
   }

}
