package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;

public class VariousWalkingProviders
{
   private final PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber;
   private final FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;

   private final FootstepProvider footstepProvider;
   private final AbortWalkingMessageSubscriber abortWalkingMessageSubscriber;

   private final HighLevelStateMessageSubscriber highLevelStateMessageSubscriber;

   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final CapturabilityBasedStatusProducer capturabilityBasedStatusProducer;

   public VariousWalkingProviders(PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber,
         FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber, StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber,
         FootstepProvider footstepProvider,
         HighLevelStateMessageSubscriber highLevelStateMessageSubscriber, CapturabilityBasedStatusProducer capturabilityBasedStatusProducer,
         AbortWalkingMessageSubscriber abortWalkingMessageSubscriber)
   {
      this.pelvisTrajectoryMessageSubscriber = pelvisTrajectoryMessageSubscriber;
      this.footTrajectoryMessageSubscriber = footTrajectoryMessageSubscriber;
      this.stopAllTrajectoryMessageSubscriber = stopAllTrajectoryMessageSubscriber;

      this.highLevelStateMessageSubscriber = highLevelStateMessageSubscriber;
      this.footstepProvider = footstepProvider;

      this.capturabilityBasedStatusProducer = capturabilityBasedStatusProducer;

      if (abortWalkingMessageSubscriber == null)
      {
         this.abortWalkingMessageSubscriber = new AbortWalkingMessageSubscriber();
      }
      else
      {
         this.abortWalkingMessageSubscriber = abortWalkingMessageSubscriber;
      }
   }

   public void clearPoseProviders()
   {
      if (pelvisTrajectoryMessageSubscriber != null)
         pelvisTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (footTrajectoryMessageSubscriber != null)
         footTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (stopAllTrajectoryMessageSubscriber != null)
         stopAllTrajectoryMessageSubscriber.clearMessagesInQueue();
   }

   public PelvisTrajectoryMessageSubscriber getPelvisTrajectoryMessageSubscriber()
   {
      return pelvisTrajectoryMessageSubscriber;
   }

   public FootTrajectoryMessageSubscriber getFootTrajectoryMessageSubscriber()
   {
      return footTrajectoryMessageSubscriber;
   }

   public StopAllTrajectoryMessageSubscriber getStopAllTrajectoryMessageSubscriber()
   {
      return stopAllTrajectoryMessageSubscriber;
   }

   public HighLevelStateMessageSubscriber getHighLevelStateMessageSubscriber()
   {
      return highLevelStateMessageSubscriber;
   }

   public FootstepProvider getFootstepProvider()
   {
      return footstepProvider;
   }

   public CapturabilityBasedStatusProducer getCapturabilityBasedStatusProducer()
   {
      return capturabilityBasedStatusProducer;
   }

   public AbortWalkingMessageSubscriber getAbortWalkingMessageSubscriber()
   {
      return abortWalkingMessageSubscriber;
   }
}
