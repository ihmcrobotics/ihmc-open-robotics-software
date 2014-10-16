package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.packets.walking.FootstepStatus.Status;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotSide.RobotSide;
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
      outgoingFootstepDataList = footsepDataList;
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
	   if(!isInitialized)
	   {
		   packetHasBeenSent.set(false);
		   
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
   
   @Override
   public boolean hasInputBeenSet() {
	   if (numberOfFootsteps.getIntegerValue() != -1 && lastFootstepStatus != null)
		   return true;
	   else
		   return false;
   }
   
}
