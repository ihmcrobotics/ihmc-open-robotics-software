package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.communication.packets.dataobjects.BlindWalkingSpeed;
import us.ihmc.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.communication.packets.walking.FootstepStatus;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

public class FootstepPathCoordinator implements FootstepProvider
{
   private final boolean DEBUG = false;

   private final FootstepTimingParameters footstepTimingParameters;

   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepPathCoordinator");
   private final EnumYoVariable<WalkMethod> walkMethod = new EnumYoVariable<WalkMethod>("walkMethod", registry, WalkMethod.class);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final GlobalDataProducer footstepStatusDataProducer;
   private Footstep stepInProgress = null;

   private final IntegerYoVariable currentFootstepIndex = new IntegerYoVariable("currentFootstepIndex", registry);

   private final BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator;
   private final DesiredFootstepCalculatorFootstepProviderWrapper desiredFootstepCalculatorFootstepProviderWrapper;
   private final ConstantSwingTimeCalculator constantSwingTimeCalculator;
   private final ConstantTransferTimeCalculator constantTransferTimeCalculator;

   private final Point3d actualFootPositionInWorld;
   private final Quat4d actualFootOrientationInWorld;

   public FootstepPathCoordinator(FootstepTimingParameters footstepTimingParameters, GlobalDataProducer objectCommunicator,
         BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator,
         ConstantSwingTimeCalculator constantSwingTimeCalculator, ConstantTransferTimeCalculator constantTransferTimeCalculator,
         YoVariableRegistry parentRegistry)
   {
      this.footstepTimingParameters = footstepTimingParameters;
      setWalkMethod(WalkMethod.FOOTSTEP_PATH);
      setPaused(false);

      this.blindWalkingToDestinationDesiredFootstepCalculator = blindWalkingToDestinationDesiredFootstepCalculator;
      this.constantSwingTimeCalculator = constantSwingTimeCalculator;
      this.constantTransferTimeCalculator = constantTransferTimeCalculator;

      footstepStatusDataProducer = objectCommunicator;

      desiredFootstepCalculatorFootstepProviderWrapper = new DesiredFootstepCalculatorFootstepProviderWrapper(
            blindWalkingToDestinationDesiredFootstepCalculator, registry);
      desiredFootstepCalculatorFootstepProviderWrapper.setWalk(true);
      currentFootstepIndex.set(-1);

      actualFootPositionInWorld = new Point3d();
      actualFootOrientationInWorld = new Quat4d();

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setSwingTime(double swingTime)
   {
      footstepTimingParameters.setSwingTime(swingTime);
   }

   public void setTransferTime(double transferTime)
   {
      footstepTimingParameters.setTransferTime(transferTime);
   }

   @Override
   public Footstep poll()
   {
      if (isPaused.getBooleanValue())
      {
         return null;
      }

      determineStepInProgress();

      if (stepInProgress != null)
      {
         if (DEBUG)
         {
            System.out.println("stepInProgress= " + stepInProgress);
         }

         notifyConsumersOfStatus(FootstepStatus.Status.STARTED, null);
      }

      return stepInProgress;
   }

   private void determineStepInProgress()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP:
         {
            stepInProgress = null;

            break;
         }

         case BLIND:
         {
            stepInProgress = desiredFootstepCalculatorFootstepProviderWrapper.poll();

            break;
         }

         case FOOTSTEP_PATH:
         {
            stepInProgress = footstepQueue.poll();

            break;
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   @Override
   public Footstep peek()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP:
         {
            return footstepQueue.peek();
         }

         case BLIND:
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.peek();
         }

         case FOOTSTEP_PATH:
         {
            return footstepQueue.peek();
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   @Override
   public Footstep peekPeek()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP:
         {
            return peekPeekUsingFootstepQueue();
         }

         case BLIND:
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.peekPeek();
         }

         case FOOTSTEP_PATH:
         {
            return peekPeekUsingFootstepQueue();
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   private Footstep peekPeekUsingFootstepQueue()
   {
      Iterator<Footstep> iterator = footstepQueue.iterator();

      if (iterator.hasNext())
      {
         iterator.next();
      }
      else
      {
         return null;
      }

      if (iterator.hasNext())
      {
         return iterator.next();
      }
      else
      {
         return null;
      }
   }

   private void notifyConsumersOfStatus(FootstepStatus.Status status, FramePose actualFootPoseInWorld)
   {
      if (footstepStatusDataProducer != null)
      {
         FootstepStatus footstepStatus;

         if (actualFootPoseInWorld != null)
         {
            actualFootPoseInWorld.getPosition(actualFootPositionInWorld);
            actualFootPoseInWorld.getOrientation(actualFootOrientationInWorld);
            footstepStatus = new FootstepStatus(status, currentFootstepIndex.getIntegerValue(), actualFootPositionInWorld, actualFootOrientationInWorld);
         }
         else
         {
            footstepStatus = new FootstepStatus(status, currentFootstepIndex.getIntegerValue());
         }

         footstepStatusDataProducer.queueDataToSend(footstepStatus);
      }
   }

   @Override
   public boolean isEmpty()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP:
         {
            return true;
         }

         case BLIND:
         {
            return isPaused.getBooleanValue() || desiredFootstepCalculatorFootstepProviderWrapper.isEmpty();
         }

         case FOOTSTEP_PATH:
         {
            return footstepQueue.isEmpty() || isPaused.getBooleanValue();
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   @Override
   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
      if (stepInProgress != null)
      {
         notifyConsumersOfStatus(FootstepStatus.Status.COMPLETED, actualFootPoseInWorld);
         currentFootstepIndex.increment();
      }
   }

   @Override
   public void notifyWalkingComplete()
   {
      if (footstepStatusDataProducer != null)
      {
         FootstepStatus footstepStatus = FootstepStatus.createWalkingIsDonePacket();
         footstepStatusDataProducer.queueDataToSend(footstepStatus);
      }
   }

   public void notifyWalkingHasBeenCancelled()
   {
      // FIXME need to report properly that the plan has been cancelled instead of calling notifyWalkingComplete().
      notifyWalkingComplete();
   }

   public void updatePath(ArrayList<Footstep> footsteps)
   {
      setWalkMethod(WalkMethod.FOOTSTEP_PATH);

      constantSwingTimeCalculator.setSwingTime(footstepTimingParameters.getFootstepPathSwingTime());
      constantTransferTimeCalculator.setTransferTime(footstepTimingParameters.getFootstepPathTransferTime());

      if (DEBUG)
      {
         System.out.println("clearing queue\n" + footstepQueue);
      }

      footstepQueue.clear();
      footstepQueue.addAll(footsteps);
      currentFootstepIndex.set(0);

      if (DEBUG)
      {
         System.out.println("new queue\n" + footstepQueue);
      }

      setPaused(false);
   }

   public void setPaused(Boolean isPaused)
   {
      if (this.isPaused.getBooleanValue() == isPaused)
      {
         return;
      }

      this.isPaused.set(isPaused);

      if (DEBUG)
      {
         System.out.println("FootstepPathCoordinator: isPaused = " + isPaused);
      }
   }

   @Override
   public boolean isPaused()
   {
      return isPaused.getBooleanValue();
   }

   public void setWalkMethod(WalkMethod walkMethod)
   {
      if (walkMethod == null)
         walkMethod = WalkMethod.STOP;
      this.walkMethod.set(walkMethod);
   }

   public void close()
   {
   }

   @Override
   public int getNumberOfFootstepsToProvide()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP:
         {
            return 0;
         }

         case BLIND:
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.getNumberOfFootstepsToProvide();
         }

         case FOOTSTEP_PATH:
         {
            return footstepQueue.size();
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   public void setBlindWalking(BlindWalkingPacket blindWalkingPacket)
   {
      FramePoint2d desiredDestination = new FramePoint2d(ReferenceFrame.getWorldFrame(), blindWalkingPacket.getDesiredDestination());
      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredDestination(desiredDestination);

      BlindWalkingDirection blindWalkingDirection = blindWalkingPacket.getBlindWalkingDirection();
      BlindWalkingSpeed blindWalkingSpeed = blindWalkingPacket.getBlindWalkingSpeed();
      boolean isInMud = blindWalkingPacket.getIsInMud();

      blindWalkingToDestinationDesiredFootstepCalculator.setBlindWalkingDirection(blindWalkingDirection);

      double stepLength;
      double stepWidth;
      double stepSideward;
      double swingTime;
      double transferTime;

      System.out.println("is In Mud = " + isInMud);

      switch (blindWalkingSpeed)
      {
         case SLOW:
         {
            if (isInMud)
            {
               stepLength = 0.25;
               stepWidth = 0.12;
               stepSideward = 0.1;
               swingTime = footstepTimingParameters.getBlindWalkingInMudSwingTime();
               transferTime = footstepTimingParameters.getBlindWalkingInMudTransferTime();
            }
            else
            {
               stepLength = 0.2;
               stepWidth = 0.25;
               stepSideward = 0.1;
               swingTime = footstepTimingParameters.getSlowBlindWalkingSwingTime();
               transferTime = footstepTimingParameters.getSlowBlindWalkingTransferTime();
            }

            break;
         }

         case MEDIUM:
         {
            if (isInMud)
            {
               stepLength = 0.35;
               stepWidth = 0.12;
               stepSideward = 0.3;
               swingTime = footstepTimingParameters.getBlindWalkingInMudSwingTime();
               transferTime = footstepTimingParameters.getBlindWalkingInMudTransferTime();
            }
            else
            {
               stepLength = 0.35;
               stepWidth = 0.25;
               stepSideward = 0.25;
               swingTime = footstepTimingParameters.getFootstepPathSwingTime();
               transferTime = footstepTimingParameters.getFootstepPathTransferTime();
            }

            break;
         }

         case FAST:
         {
            if (isInMud)
            {
               stepLength = 0.5;
               stepWidth = 0.12;
               stepSideward = 0.5;
               swingTime = footstepTimingParameters.getBlindWalkingInMudSwingTime();
               transferTime = footstepTimingParameters.getBlindWalkingInMudTransferTime();
            }
            else
            {
               stepLength = 0.5;
               stepWidth = 0.25;
               stepSideward = 0.6;
               swingTime = footstepTimingParameters.getFootstepPathSwingTime();
               transferTime = footstepTimingParameters.getFootstepPathTransferTime();
            }

            break;
         }

         default:
         {
            stepLength = 0.0;
            stepWidth = 0.15;
            stepSideward = 0.0;
            swingTime = footstepTimingParameters.getSlowBlindWalkingSwingTime();
            transferTime = footstepTimingParameters.getSlowBlindWalkingTransferTime();

            break;
         }
      }

      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepWidth(stepWidth);
      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepForward(stepLength);
      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepSideward(stepSideward);
      constantSwingTimeCalculator.setSwingTime(swingTime);
      constantTransferTimeCalculator.setTransferTime(transferTime);

      setWalkMethod(WalkMethod.BLIND);
      footstepQueue.clear();
      setPaused(false);
   }

   private enum WalkMethod
   {
      STOP, FOOTSTEP_PATH, BLIND;
   }

   @Override
   public boolean isBlindWalking()
   {
      if (walkMethod.getEnumValue() == WalkMethod.BLIND)
         return true;

      return false;
   }

   @Override
   public void cancelPlan()
   {
      notifyWalkingHasBeenCancelled();
      switch (walkMethod.getEnumValue())
      {
         case BLIND:
            desiredFootstepCalculatorFootstepProviderWrapper.cancelPlan();
            break;
         case FOOTSTEP_PATH:
            footstepQueue.clear();
            break;
      }
   }
}
