package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

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
   private final BooleanYoVariable isDone = new BooleanYoVariable("isDone", registry);
   private final BooleanYoVariable hasLastStepBeenReached = new BooleanYoVariable("hasLastStepBeenReached", registry);
   private final BooleanYoVariable footStepStatusIsDoneWalking = new BooleanYoVariable("footStepStatusIsDoneWalking", registry);

   private double defaultSwingTime;
   private double defaultTranferTime;

   public FootstepListBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);
      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      attachControllerListeningQueue(footstepStatusQueue, FootstepStatus.class);
      numberOfFootsteps.set(-1);
      defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      defaultTranferTime = walkingControllerParameters.getDefaultTransferTime();
   }

   public void set(FootstepDataList footStepList)
   {
      outgoingFootstepDataList = footStepList;
      numberOfFootsteps.set(outgoingFootstepDataList.getDataList().size());
      packetHasBeenSent.set(false);
   }
   
   public void set(ArrayList<Footstep> footsteps, double swingTime, double transferTime)
   {
      FootstepDataList footstepDataList = new FootstepDataList(swingTime,transferTime);
      
      for (int i = 0; i < footsteps.size(); i++)
      {
         Footstep footstep = footsteps.get(i);
         Point3d location = new Point3d(footstep.getX(), footstep.getY(), footstep.getZ());
         Quat4d orientation = new Quat4d();
         footstep.getOrientation(orientation);
         
         RobotSide footstepSide = footstep.getRobotSide();
         FootstepDataMessage footstepData = new FootstepDataMessage(footstepSide, location, orientation);
         footstepDataList.add(footstepData);
      }
      set(footstepDataList);
   }
   
   public void set(ArrayList<Footstep> footsteps)
   {
      set(footsteps, defaultSwingTime, defaultTranferTime);
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
         outgoingFootstepDataList.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(outgoingFootstepDataList);

         outgoingFootstepDataList.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(outgoingFootstepDataList);
         packetHasBeenSent.set(true);
      }
   }

   private void checkForNewFootstepStatusPacket()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         FootstepStatus newestFootstepStatus = footstepStatusQueue.getNewestPacket();
         if (newestFootstepStatus != null)
         {
            lastFootstepStatus = newestFootstepStatus;

            int currentStepIndex = lastFootstepStatus.getFootstepIndex();
            int currentStepNumber = currentStepIndex + 1;

            if (currentStepNumber >= numberOfFootsteps.getIntegerValue())
               hasLastStepBeenReached.set(true);

            if (lastFootstepStatus.isDoneWalking())
               footStepStatusIsDoneWalking.set(true);

            if (DEBUG)
            {
               PrintTools.debug(this, "** \n Recieved new FootStepStatus : " + lastFootstepStatus);
               PrintTools.debug(this, "current footstep number: " + currentStepNumber + ", total number of footsteps: " + numberOfFootsteps.getIntegerValue());
               PrintTools.debug(this, "Reached last step? : " + hasLastStepBeenReached.getBooleanValue());
               PrintTools.debug(this, "FootstepStatus: is done walking? : " + footStepStatusIsDoneWalking.getBooleanValue());

               if (!isDone.getBooleanValue() && isDone())
               {
                  PrintTools.debug(this, "*****  Setting isDone = true  *******");
               }
               else
               {
                  PrintTools.debug(this, "is Behavior done? : " + isDone.getBooleanValue());
               }
            }
         }
      }
   }

   @Override
   public void initialize()
   {
      if (DEBUG)
         PrintTools.debug(this, "Initialize");
      packetHasBeenSent.set(false);
      hasLastStepBeenReached.set(false);
      footStepStatusIsDoneWalking.set(false);

      isPaused.set(false);
      isStopped.set(false);
      hasBeenInitialized.set(true);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      if (DEBUG)
         PrintTools.debug(this, "Finalize");
      footstepStatusQueue.clear();
      outgoingFootstepDataList = null;
      packetHasBeenSent.set(false);
      numberOfFootsteps.set(-1);

      lastFootstepStatus = null;
      isPaused.set(false);
      isStopped.set(false);
      hasBeenInitialized.set(false);
      hasLastStepBeenReached.set(false);
      footStepStatusIsDoneWalking.set(false);
   }

   @Override
   public void stop()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      isStopped.set(true);
   }

   @Override
   public void pause()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      isPaused.set(true);
      if (DEBUG)
         PrintTools.debug(this, "Pausing Behavior");
   }

   @Override
   public void resume()
   {
      sendPacketToController(new PauseWalkingMessage(false));
      isPaused.set(false);
      isStopped.set(false);
      if (DEBUG)
         PrintTools.debug(this, "Resuming Behavior");
   }

   @Override
   public boolean isDone()
   {
      boolean ret = footStepStatusIsDoneWalking.getBooleanValue() && hasLastStepBeenReached.getBooleanValue() && !isPaused.getBooleanValue();
      if (!isDone.getBooleanValue() && ret)
      {
         if (DEBUG)
            PrintTools.debug(this, "*****  Setting isDone = true  *******");
      }
      isDone.set(ret);
      return ret;
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

   private final ArrayList<FootstepDataMessage> footstepDataList = new ArrayList<FootstepDataMessage>();
   private final Vector3d firstSingleSupportFootTranslationFromWorld = new Vector3d();
   private final Point3d previousFootStepLocation = new Point3d();
   private final Point3d nextFootStepLocation = new Point3d();

   public ArrayList<Double> getFootstepLengths(FootstepDataList footStepList, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters)
   {
      ArrayList<Double> footStepLengths = new ArrayList<Double>();
      footstepDataList.addAll(footStepList.getDataList());

      FootstepDataMessage firstStepData = footstepDataList.remove(footstepDataList.size() - 1);

      RigidBodyTransform firstSingleSupportFootTransformToWorld = fullRobotModel.getFoot(firstStepData.getRobotSide().getOppositeSide()).getBodyFixedFrame()
            .getTransformToWorldFrame();
      firstSingleSupportFootTransformToWorld.getTranslation(firstSingleSupportFootTranslationFromWorld);

      previousFootStepLocation.set(firstSingleSupportFootTranslationFromWorld);
      firstStepData.getLocation(nextFootStepLocation);

      while (!footstepDataList.isEmpty())
      {
         footStepLengths.add(previousFootStepLocation.distance(nextFootStepLocation));
         previousFootStepLocation.set(nextFootStepLocation);
         footstepDataList.remove(footstepDataList.size() - 1).getLocation(nextFootStepLocation);
      }

      double lastStepLength = previousFootStepLocation.distance(nextFootStepLocation);
      footStepLengths.add(lastStepLength);

      return footStepLengths;
   }

   public boolean areFootstepsTooFarApart(FootstepDataList footStepList, FullHumanoidRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters)
   {
      for (double stepLength : getFootstepLengths(footStepList, fullRobotModel, walkingControllerParameters))
      {
         if (DEBUG)
            PrintTools.debug(this, "step length : " + stepLength + " max step length : " + walkingControllerParameters.getMaxStepLength());
         if (stepLength > walkingControllerParameters.getMaxStepLength())
         {
            return true;
         }
      }

      return false;
   }

}
