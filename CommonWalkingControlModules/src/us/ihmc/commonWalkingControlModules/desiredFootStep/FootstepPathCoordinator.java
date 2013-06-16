package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingDirection;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingSpeed;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class FootstepPathCoordinator implements FootstepProvider
{
   private static final double FOOTSTEP_PATH_SWING_TIME = 0.6;
   private static final double SLOW_BLIND_WALKING_SWING_TIME = 0.8;
   private static final double BLIND_WALKING_IN_MUD_SWING_TIME = 2.2; //1.6; //2.5;
   
   private static final double FOOTSTEP_PATH_TRANSFER_TIME = 0.25;
   private static final double SLOW_BLIND_WALKING_TRANSFER_TIME = 0.35;
   private static final double BLIND_WALKING_IN_MUD_TRANSFER_TIME = 0.6; //1.5;
   
   private boolean DEBUG = false;
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepPathCoordinator");
   private final EnumYoVariable<WalkMethod> walkMethod = new EnumYoVariable<WalkMethod>("walkMethod", registry, WalkMethod.class);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final QueueBasedStreamingDataProducer<FootstepStatus> footstepStatusDataProducer;
   private Footstep stepInProgress = null;

   private final BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator;
   private final DesiredFootstepCalculatorFootstepProviderWrapper desiredFootstepCalculatorFootstepProviderWrapper;
   private final ConstantSwingTimeCalculator constantSwingTimeCalculator;
   private final ConstantTransferTimeCalculator constantTransferTimeCalculator;

   public FootstepPathCoordinator(ObjectCommunicator objectCommunicator,
                                  BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator,
                                  ConstantSwingTimeCalculator constantSwingTimeCalculator,
                                  ConstantTransferTimeCalculator constantTransferTimeCalculator,
                                  YoVariableRegistry parentRegistry)
   {
      setWalkMethod(WalkMethod.FOOTSTEP_PATH);
      setPaused(false);

      this.blindWalkingToDestinationDesiredFootstepCalculator = blindWalkingToDestinationDesiredFootstepCalculator;
      this.constantSwingTimeCalculator = constantSwingTimeCalculator;
      this.constantTransferTimeCalculator = constantTransferTimeCalculator;
      
      footstepStatusDataProducer = new QueueBasedStreamingDataProducer<FootstepStatus>();

      if (objectCommunicator != null)
      {
         footstepStatusDataProducer.addConsumer(objectCommunicator);
      }

      footstepStatusDataProducer.startProducingData();

      desiredFootstepCalculatorFootstepProviderWrapper =
         new DesiredFootstepCalculatorFootstepProviderWrapper(blindWalkingToDestinationDesiredFootstepCalculator, registry);
      desiredFootstepCalculatorFootstepProviderWrapper.setWalk(true);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

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

         notifyConsumersOfStatus(FootstepStatus.Status.STARTED);
      }

      return stepInProgress;
   }

   private void determineStepInProgress()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP :
         {
            stepInProgress = null;

            break;
         }

         case BLIND :
         {
            stepInProgress = desiredFootstepCalculatorFootstepProviderWrapper.poll();

            break;
         }

         case FOOTSTEP_PATH :
         {
            stepInProgress = footstepQueue.poll();

            break;
         }

         default :
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   public Footstep peek()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP :
         {
            return footstepQueue.peek();
         }

         case BLIND :
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.peek();
         }

         case FOOTSTEP_PATH :
         {
            return footstepQueue.peek();
         }

         default :
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   public Footstep peekPeek()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP :
         {
            return peekPeekUsingFootstepQueue();
         }

         case BLIND :
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.peekPeek();
         }

         case FOOTSTEP_PATH :
         {
            return peekPeekUsingFootstepQueue();
         }

         default :
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

   private void notifyConsumersOfStatus(FootstepStatus.Status status)
   {
      FootstepStatus footstepStatus = new FootstepStatus(status);
      footstepStatusDataProducer.queueDataToSend(footstepStatus);
   }

   public boolean isEmpty()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP :
         {
            return true;
         }

         case BLIND :
         {
            return isPaused.getBooleanValue() || desiredFootstepCalculatorFootstepProviderWrapper.isEmpty();
         }

         case FOOTSTEP_PATH :
         {
            return footstepQueue.isEmpty() || isPaused.getBooleanValue();
         }

         default :
         {
            throw new RuntimeException("Shouldn't get here!");
         }
      }
   }

   public void notifyComplete()
   {
      if (stepInProgress != null)
      {
         notifyConsumersOfStatus(FootstepStatus.Status.COMPLETED);
      }
   }

   public void updatePath(ArrayList<Footstep> footsteps)
   {
      setWalkMethod(WalkMethod.FOOTSTEP_PATH);

      constantSwingTimeCalculator.setSwingTime(FOOTSTEP_PATH_SWING_TIME);
      constantTransferTimeCalculator.setTransferTime(FOOTSTEP_PATH_TRANSFER_TIME);
      
      if (DEBUG)
      {
         System.out.println("clearing queue\n" + footstepQueue);
      }

      footstepQueue.clear();
      footstepQueue.addAll(footsteps);

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

   public void setWalkMethod(WalkMethod walkMethod)
   {
      if (walkMethod == null)
         walkMethod = WalkMethod.STOP;
      this.walkMethod.set(walkMethod);
   }

   public void close()
   {
   }

   public int getNumberOfFootstepsToProvide()
   {
      switch (walkMethod.getEnumValue())
      {
         case STOP :
         {
            return 0;
         }

         case BLIND :
         {
            return desiredFootstepCalculatorFootstepProviderWrapper.getNumberOfFootstepsToProvide();
         }

         case FOOTSTEP_PATH :
         {
            return footstepQueue.size();
         }

         default :
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
         case SLOW :
         {
            if (isInMud)
            {
               stepLength = 0.25; 
               stepWidth = 0.15;
               stepSideward = 0.1;
               swingTime = BLIND_WALKING_IN_MUD_SWING_TIME;
               transferTime = BLIND_WALKING_IN_MUD_TRANSFER_TIME; 
            }
            else
            {
               stepLength = 0.2;
               stepWidth = 0.25;
               stepSideward = 0.1;
               swingTime = SLOW_BLIND_WALKING_SWING_TIME;
               transferTime = SLOW_BLIND_WALKING_TRANSFER_TIME; 
            }

            break;
         }

         case MEDIUM :
         {
            if (isInMud)
            {
               stepLength = 0.35;
               stepWidth = 0.15;
               stepSideward = 0.3;
               swingTime = BLIND_WALKING_IN_MUD_SWING_TIME;
               transferTime = BLIND_WALKING_IN_MUD_TRANSFER_TIME; 
            }
            else
            {
               stepLength = 0.35;
               stepWidth = 0.25;
               stepSideward = 0.25;
               swingTime = FOOTSTEP_PATH_SWING_TIME;
               transferTime = FOOTSTEP_PATH_TRANSFER_TIME;  
            }

            break;
         }

         case FAST :
         { 
            if (isInMud)
            {
               stepLength = 0.5;
               stepWidth = 0.15;
               stepSideward = 0.5;
               swingTime = BLIND_WALKING_IN_MUD_SWING_TIME;
               transferTime = BLIND_WALKING_IN_MUD_TRANSFER_TIME; 
            }
            else
            {
               stepLength = 0.5;
               stepWidth = 0.25;
               stepSideward = 0.6;
               swingTime = FOOTSTEP_PATH_SWING_TIME;
               transferTime = FOOTSTEP_PATH_TRANSFER_TIME;
            }

            break;
         }

         default :
         {
            stepLength = 0.0;
            stepWidth = 0.15;
            stepSideward = 0.0;
            swingTime = SLOW_BLIND_WALKING_SWING_TIME;
            transferTime = SLOW_BLIND_WALKING_TRANSFER_TIME;

            break;
         }
      }

      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepWidth(stepWidth);
      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepForward(stepLength);
      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepSideward(stepSideward);
      constantSwingTimeCalculator.setSwingTime(swingTime);
      constantTransferTimeCalculator.setTransferTime(transferTime);

//    blindWalkingToDestinationDesiredFootstepCalculator.setInPlaceWidth(stepWidth);

      setWalkMethod(WalkMethod.BLIND);
      footstepQueue.clear();
      setPaused(false);

   }

   private enum WalkMethod {STOP, FOOTSTEP_PATH, BLIND;}

   public boolean isBlindWalking()
   {
      if (walkMethod.getEnumValue() == WalkMethod.BLIND) return true;
      return false;
   }
}
