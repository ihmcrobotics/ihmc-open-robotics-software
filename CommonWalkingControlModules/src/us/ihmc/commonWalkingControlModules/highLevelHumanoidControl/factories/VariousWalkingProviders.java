package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmDesiredAccelerationsMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.GoHomeMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisHeightTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisOrientationTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;

public class VariousWalkingProviders
{
   private final HandTrajectoryMessageSubscriber handTrajectoryMessageSubscriber;
   private final ArmTrajectoryMessageSubscriber armTrajectoryMessageSubscriber;
   private final ArmDesiredAccelerationsMessageSubscriber armDesiredAccelerationsMessageSubscriber;
   private final HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber;
   private final ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber;
   private final PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber;
   private final PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber;
   private final FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber;
   private final EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final PelvisHeightTrajectoryMessageSubscriber pelvisHeightTrajectoryMessageSubscriber;

   private final HandComplianceControlParametersSubscriber handComplianceControlParametersSubscriber;
   private final GoHomeMessageSubscriber goHomeMessageSubscriber;

   private final FootstepProvider footstepProvider;
   private final AbortWalkingMessageSubscriber abortWalkingMessageSubscriber;

   private final AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator;

   private final HighLevelStateMessageSubscriber highLevelStateMessageSubscriber;

   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final CapturabilityBasedStatusProducer capturabilityBasedStatusProducer;
   private final HandPoseStatusProducer handPoseStatusProducer;

   public VariousWalkingProviders(HandTrajectoryMessageSubscriber handTrajectoryMessageSubscriber,
         ArmTrajectoryMessageSubscriber armTrajectoryMessageSubscriber, ArmDesiredAccelerationsMessageSubscriber armDesiredAccelerationsMessageSubscriber,
         HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber, ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber,
         PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber,
         PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber,
         FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber, EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber,
         StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber, PelvisHeightTrajectoryMessageSubscriber pelvisHeightTrajectoryMessageSubscriber,
         GoHomeMessageSubscriber goHomeMessageSubscriber, FootstepProvider footstepProvider,
         HandComplianceControlParametersSubscriber handComplianceControlParametersSubscriber,
         AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator, HighLevelStateMessageSubscriber highLevelStateMessageSubscriber,
         CapturabilityBasedStatusProducer capturabilityBasedStatusProducer, HandPoseStatusProducer handPoseStatusProducer,
         AbortWalkingMessageSubscriber abortWalkingMessageSubscriber)
   {
      this.handTrajectoryMessageSubscriber = handTrajectoryMessageSubscriber;
      this.armTrajectoryMessageSubscriber = armTrajectoryMessageSubscriber;
      this.armDesiredAccelerationsMessageSubscriber = armDesiredAccelerationsMessageSubscriber;
      this.headTrajectoryMessageSubscriber = headTrajectoryMessageSubscriber;
      this.chestTrajectoryMessageSubscriber = chestTrajectoryMessageSubscriber;
      this.pelvisTrajectoryMessageSubscriber = pelvisTrajectoryMessageSubscriber;
      this.pelvisOrientationTrajectoryMessageSubscriber = pelvisOrientationTrajectoryMessageSubscriber;
      this.footTrajectoryMessageSubscriber = footTrajectoryMessageSubscriber;
      this.endEffectorLoadBearingMessageSubscriber = endEffectorLoadBearingMessageSubscriber;
      this.stopAllTrajectoryMessageSubscriber = stopAllTrajectoryMessageSubscriber;
      this.pelvisHeightTrajectoryMessageSubscriber = pelvisHeightTrajectoryMessageSubscriber;
      this.goHomeMessageSubscriber = goHomeMessageSubscriber;

      this.highLevelStateMessageSubscriber = highLevelStateMessageSubscriber;
      this.footstepProvider = footstepProvider;
      this.handComplianceControlParametersSubscriber = handComplianceControlParametersSubscriber;

      this.automaticManipulationAbortCommunicator = automaticManipulationAbortCommunicator;

      this.capturabilityBasedStatusProducer = capturabilityBasedStatusProducer;

      this.handPoseStatusProducer = handPoseStatusProducer;

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
      if (handTrajectoryMessageSubscriber != null)
         handTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (armTrajectoryMessageSubscriber != null)
         armTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (armDesiredAccelerationsMessageSubscriber != null)
         armDesiredAccelerationsMessageSubscriber.clearMessagesInQueue();
      if (headTrajectoryMessageSubscriber != null)
         headTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (chestTrajectoryMessageSubscriber != null)
         chestTrajectoryMessageSubscriber.clearMessagesInQueue();
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
      if (pelvisHeightTrajectoryMessageSubscriber != null)
         pelvisHeightTrajectoryMessageSubscriber.clearMessagesInQueue();
      if (goHomeMessageSubscriber != null)
         goHomeMessageSubscriber.clearMessagesInQueue();
   }

   public HandTrajectoryMessageSubscriber getHandTrajectoryMessageSubscriber()
   {
      return handTrajectoryMessageSubscriber;
   }

   public ArmTrajectoryMessageSubscriber geArmTrajectoryMessageSubscriber()
   {
      return armTrajectoryMessageSubscriber;
   }

   public ArmDesiredAccelerationsMessageSubscriber getArmDesiredAccelerationsMessageSubscriber()
   {
      return armDesiredAccelerationsMessageSubscriber;
   }

   public HeadTrajectoryMessageSubscriber getHeadTrajectoryMessageSubscriber()
   {
      return headTrajectoryMessageSubscriber;
   }

   public ChestTrajectoryMessageSubscriber getChestTrajectoryMessageSubscriber()
   {
      return chestTrajectoryMessageSubscriber;
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

   public PelvisHeightTrajectoryMessageSubscriber getPelvisHeightTrajectoryMessageSubscriber()
   {
      return pelvisHeightTrajectoryMessageSubscriber;
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

   public HandComplianceControlParametersSubscriber getHandComplianceControlParametersSubscriber()
   {
      return handComplianceControlParametersSubscriber;
   }

   public CapturabilityBasedStatusProducer getCapturabilityBasedStatusProducer()
   {
      return capturabilityBasedStatusProducer;
   }

   public HandPoseStatusProducer getHandPoseStatusProducer()
   {
      return handPoseStatusProducer;
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
