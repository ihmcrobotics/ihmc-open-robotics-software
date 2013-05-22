package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * User: Matt
 * Date: 1/17/13
 */
public class FootstepPathCoordinator implements FootstepProvider
{
   private boolean DEBUG = false;
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private YoVariableRegistry registry = new YoVariableRegistry("FootstepPathCoordinator");
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final QueueBasedStreamingDataProducer<FootstepStatus> footstepStatusDataProducer;
   private Footstep stepInProgress = null;

   
   public FootstepPathCoordinator()
   {
      this(null);
   }
   
   public FootstepPathCoordinator(ObjectCommunicator objectCommunicator)
   {
      setPaused(false);
      setWalk(true);

      footstepStatusDataProducer = new QueueBasedStreamingDataProducer<FootstepStatus>();
      if(objectCommunicator != null)
      {
         footstepStatusDataProducer.addConsumer(objectCommunicator);
      }
      footstepStatusDataProducer.startProducingData();
   }

   public FootstepPathCoordinator(ObjectCommunicator objectCommunicator, YoVariableRegistry parentRegistry)
   {
      this(objectCommunicator);
      parentRegistry.addChild(registry);
   }

   public Footstep poll()
   {
      if (isPaused.getBooleanValue())
      {
         return null;
      }

      stepInProgress = footstepQueue.poll();

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

   public Footstep peek()
   {
      return footstepQueue.peek();
   }
   
   public Footstep peekPeek()
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
      return (!walk.getBooleanValue() || footstepQueue.isEmpty() || isPaused.getBooleanValue());
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

   public void setWalk(boolean walk)
   {
      this.walk.set(walk);
   }

   public void close()
   {
   }

   public int getNumberOfFootstepsToProvide()
   {
      return footstepQueue.size();
   }
}
