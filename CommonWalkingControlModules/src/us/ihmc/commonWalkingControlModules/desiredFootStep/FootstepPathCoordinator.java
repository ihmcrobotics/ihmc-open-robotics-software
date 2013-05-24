package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingDirection;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingSpeed;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class FootstepPathCoordinator implements FootstepProvider
{
   private boolean DEBUG = false;
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final YoVariableRegistry registry = new YoVariableRegistry("FootstepPathCoordinator");
   private final EnumYoVariable<WalkMethod> walkMethod = new EnumYoVariable<WalkMethod>("walkMethod", registry, WalkMethod.class);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final QueueBasedStreamingDataProducer<FootstepStatus> footstepStatusDataProducer;
   private Footstep stepInProgress = null;

   private final BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator;
   private final DesiredFootstepCalculatorFootstepProviderWrapper desiredFootstepCalculatorFootstepProviderWrapper;

   public FootstepPathCoordinator()
   {
      this(null, null);
   }

   public FootstepPathCoordinator(ObjectCommunicator objectCommunicator,
                                  BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator)
   {
      this(objectCommunicator, blindWalkingToDestinationDesiredFootstepCalculator, null);
   }

   public FootstepPathCoordinator(ObjectCommunicator objectCommunicator,
                                  BlindWalkingToDestinationDesiredFootstepCalculator blindWalkingToDestinationDesiredFootstepCalculator,
                                  YoVariableRegistry parentRegistry)
   {
      setWalkMethod(WalkMethod.FOOTSTEP_PATH);
      setPaused(false);

      this.blindWalkingToDestinationDesiredFootstepCalculator = blindWalkingToDestinationDesiredFootstepCalculator;

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

      if (blindWalkingDirection == BlindWalkingDirection.BACKWARD)
      {
         blindWalkingToDestinationDesiredFootstepCalculator.setWalkBackwards(true);
      }
      else
      {
         blindWalkingToDestinationDesiredFootstepCalculator.setWalkBackwards(false);
      }

      double stepLength;
      switch (blindWalkingSpeed)
      {
         case SLOW :
         {
            stepLength = 0.2;

            break;
         }

         case MEDIUM :
         {
            stepLength = 0.35;

            break;
         }

         case FAST :
         {
            stepLength = 0.5;

            break;
         }

         default :
         {
            stepLength = 0.0;

            break;
         }
      }

      blindWalkingToDestinationDesiredFootstepCalculator.setDesiredStepForward(stepLength);

//    blindWalkingToDestinationDesiredFootstepCalculator.setInPlaceWidth(stepWidth);

      setWalkMethod(WalkMethod.BLIND);
      footstepQueue.clear();
      setPaused(false);

   }

   private enum WalkMethod {STOP, FOOTSTEP_PATH, BLIND;}
}
