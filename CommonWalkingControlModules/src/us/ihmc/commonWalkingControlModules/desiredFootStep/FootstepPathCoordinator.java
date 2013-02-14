package us.ihmc.commonWalkingControlModules.desiredFootStep;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;
import us.ihmc.utilities.io.streamingData.StreamingDataTCPServer;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

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
   private final StreamingDataTCPServer streamingDataTCPServer;
   private Footstep stepInProgress = null;

   public FootstepPathCoordinator()
   {
      setPaused(false);
      setWalk(true);

      footstepStatusDataProducer = new QueueBasedStreamingDataProducer<FootstepStatus>(4444L);
      streamingDataTCPServer = new StreamingDataTCPServer(4444);
      streamingDataTCPServer.registerStreamingDataProducer(footstepStatusDataProducer);
      streamingDataTCPServer.startOnAThread();
      footstepStatusDataProducer.startProducingData();
   }

   public FootstepPathCoordinator(YoVariableRegistry parentRegistry)
   {
      this();
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
         notifyConsumersOfStatus(stepInProgress, FootstepStatus.Status.STARTED);
      }
      return stepInProgress;
   }
   
   public Footstep peek()
   {
      return footstepQueue.peek();
   }

   private void notifyConsumersOfStatus(Footstep footstep, FootstepStatus.Status status)
   {
      FootstepStatus footstepStatus = new FootstepStatus(footstep, status);
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
         notifyConsumersOfStatus(stepInProgress, FootstepStatus.Status.COMPLETED);
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
      streamingDataTCPServer.close();
   }
}
