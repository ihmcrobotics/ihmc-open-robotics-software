package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.GoHomeMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisOrientationTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;

public class VariousWalkingProviders
{
   private final PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber;
   private final PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber;
   private final FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber;
   private final EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;

   private final GoHomeMessageSubscriber goHomeMessageSubscriber;

   private final FootstepProvider footstepProvider;
   private final AbortWalkingMessageSubscriber abortWalkingMessageSubscriber;

   private final AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator;

   private final HighLevelStateMessageSubscriber highLevelStateMessageSubscriber;

   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final CapturabilityBasedStatusProducer capturabilityBasedStatusProducer;

   public VariousWalkingProviders(PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber,
         PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber, FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber,
         EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber,
         StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber, GoHomeMessageSubscriber goHomeMessageSubscriber,
         FootstepProvider footstepProvider, AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator,
         HighLevelStateMessageSubscriber highLevelStateMessageSubscriber, CapturabilityBasedStatusProducer capturabilityBasedStatusProducer,
         AbortWalkingMessageSubscriber abortWalkingMessageSubscriber)
   {
      this.pelvisTrajectoryMessageSubscriber = pelvisTrajectoryMessageSubscriber;
      this.pelvisOrientationTrajectoryMessageSubscriber = pelvisOrientationTrajectoryMessageSubscriber;
      this.footTrajectoryMessageSubscriber = footTrajectoryMessageSubscriber;
      this.endEffectorLoadBearingMessageSubscriber = endEffectorLoadBearingMessageSubscriber;
      this.stopAllTrajectoryMessageSubscriber = stopAllTrajectoryMessageSubscriber;
      this.goHomeMessageSubscriber = goHomeMessageSubscriber;

      this.highLevelStateMessageSubscriber = highLevelStateMessageSubscriber;
      this.footstepProvider = footstepProvider;

      this.automaticManipulationAbortCommunicator = automaticManipulationAbortCommunicator;

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
      if (pelvisOrientationTrajectoryMessageSubscriber != null)
         pelvisOrientationTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (footTrajectoryMessageSubscriber != null)
         footTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (endEffectorLoadBearingMessageSubscriber != null)
         endEffectorLoadBearingMessageSubscriber.clearMessagesInQueue();
      if (stopAllTrajectoryMessageSubscriber != null)
         stopAllTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (goHomeMessageSubscriber != null)
         goHomeMessageSubscriber.clearMessagesInQueue();
   }

   public PelvisTrajectoryMessageSubscriber getPelvisTrajectoryMessageSubscriber()
   {
      return pelvisTrajectoryMessageSubscriber;
   }

   public PelvisOrientationTrajectoryMessageSubscriber getPelvisOrientationTrajectoryMessageSubscriber()
   {
      return pelvisOrientationTrajectoryMessageSubscriber;
   }

   public FootTrajectoryMessageSubscriber getFootTrajectoryMessageSubscriber()
   {
      return footTrajectoryMessageSubscriber;
   }

   public EndEffectorLoadBearingMessageSubscriber getEndEffectorLoadBearingMessageSubscriber()
   {
      return endEffectorLoadBearingMessageSubscriber;
   }

   public StopAllTrajectoryMessageSubscriber getStopAllTrajectoryMessageSubscriber()
   {
      return stopAllTrajectoryMessageSubscriber;
   }

   public GoHomeMessageSubscriber getGoHomeMessageSubscriber()
   {
      return goHomeMessageSubscriber;
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

   public AutomaticManipulationAbortCommunicator getAutomaticManipulationAbortCommunicator()
   {
      return automaticManipulationAbortCommunicator;
   }
}
