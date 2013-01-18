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
   private final ConcurrentLinkedQueue<Footstep> footstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private final ConcurrentLinkedQueue<Footstep> pausedFootstepQueue = new ConcurrentLinkedQueue<Footstep>();
   private YoVariableRegistry registry = new YoVariableRegistry("FootstepPathCoordinator");
   private final BooleanYoVariable walk = new BooleanYoVariable("walk", registry);
   private final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused", registry);
   private final QueueBasedStreamingDataProducer<FootstepStatus> footstepStatusDataProducer;
   private final StreamingDataTCPServer streamingDataTCPServer;

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
      Footstep footstep = footstepQueue.poll();
      if (footstep != null)
      {
         notifyConsumersOfStatus(footstep);
      }
      return footstep;
   }

   private void notifyConsumersOfStatus(Footstep footstep)
   {
      FootstepStatus footstepStatus = new FootstepStatus(new FootstepData(footstep), FootstepStatus.Status.STARTED);
      footstepStatusDataProducer.queueDataToSend(footstepStatus);
   }

   public boolean isEmpty()
   {
      return (!walk.getBooleanValue() || footstepQueue.isEmpty());
   }

   public void updatePath(ArrayList<Footstep> footsteps)
   {
      if (isPaused.getBooleanValue())
      {
         pausedFootstepQueue.clear();
         pausedFootstepQueue.addAll(footsteps);
      }
      else
      {
         footstepQueue.clear();
         footstepQueue.addAll(footsteps);
      }

   }

   public void setPaused(Boolean isPaused)
   {
      if (this.isPaused.getBooleanValue() == isPaused)
      {
         return;
      }

      this.isPaused.set(isPaused);

      if (isPaused)
      {
         pausedFootstepQueue.clear();
         pausedFootstepQueue.addAll(footstepQueue);
         footstepQueue.clear();
      }
      else
      {
         footstepQueue.clear();
         footstepQueue.addAll(pausedFootstepQueue);
         pausedFootstepQueue.clear();
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
