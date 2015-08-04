package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredSteeringWheelProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.MultiJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectWeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.SingleJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.robotSide.RobotSide;

public class VariousWalkingProviders
{
   private final FootstepProvider footstepProvider;
   private final HandstepProvider handstepProvider;
   private final AbortWalkingProvider abortProvider;

   private final AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator;

   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   private final DesiredHighLevelStateProvider desiredHighLevelStateProvider;
   private final HeadOrientationProvider desiredHeadOrientationProvider;
   private final PelvisPoseProvider desiredPelvisPoseProvider;
   private final DesiredComHeightProvider desiredComHeightProvider;
   private final HandPoseProvider desiredHandPoseProvider;
   private final HandComplianceControlParametersProvider handComplianceControlParametersProvider;
   private final DesiredSteeringWheelProvider desiredSteeringWheelProvider;
   private final ChestOrientationProvider desiredChestOrientationProvider;
   private final FootPoseProvider footPoseProvider;

   // TODO: Rename DesiredFootStateProvider to DesiredFootLoadBearingProvider, do the same for the packet, etc.
   private final DesiredFootStateProvider desiredFootLoadBearingProvider;
   private final HandLoadBearingProvider desiredHandLoadBearingProvider;
   private final DesiredPelvisLoadBearingProvider desiredPelvisLoadBearingProvider;
   private final DesiredThighLoadBearingProvider desiredThighLoadBearingProvider;

   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final ControlStatusProducer controlStatusProducer;

   private final CapturabilityBasedStatusProducer capturabilityBasedStatusProducer;

   private final HandPoseStatusProducer handPoseStatusProducer;

   private final ObjectWeightProvider objectWeightProvider;
   private final DesiredJointsPositionProvider desiredJointsPositionProvider;
   private final SingleJointPositionProvider singleJointPositionProvider;
   private final MultiJointPositionProvider multiJointPositionProvider;

   public VariousWalkingProviders(FootstepProvider footstepProvider, HandstepProvider handstepProvider,
                                  HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters,
                                  HeadOrientationProvider desiredHeadOrientationProvider, DesiredComHeightProvider desiredComHeightProvider,
                                  PelvisPoseProvider desiredPelvisPoseProvider, HandPoseProvider desiredHandPoseProvider,
                                  HandComplianceControlParametersProvider handComplianceControlParametersProvider, DesiredSteeringWheelProvider desiredSteeringWheelProvider,
                                  HandLoadBearingProvider desiredHandLoadBearingProvider, AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator, ChestOrientationProvider desiredChestOrientationProvider,
                                  FootPoseProvider footPoseProvider, DesiredFootStateProvider footStateProvider, DesiredHighLevelStateProvider desiredHighLevelStateProvider,
                                  DesiredThighLoadBearingProvider thighLoadBearingProvider, DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider,
                                  ControlStatusProducer controlStatusProducer, CapturabilityBasedStatusProducer capturabilityBasedStatusProducer,
                                  HandPoseStatusProducer handPoseStatusProducer, ObjectWeightProvider objectWeightProvider, DesiredJointsPositionProvider desiredJointsPositionProvider,
                                  SingleJointPositionProvider singleJointPositionProvider, AbortWalkingProvider abortProvider,
                                  MultiJointPositionProvider multiJointPositionProvider)
   {
      this.desiredHighLevelStateProvider = desiredHighLevelStateProvider;
      this.footstepProvider = footstepProvider;
      this.handstepProvider = handstepProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.desiredComHeightProvider = desiredComHeightProvider;
      this.desiredChestOrientationProvider = desiredChestOrientationProvider;
      this.desiredHandPoseProvider = desiredHandPoseProvider;
      this.desiredSteeringWheelProvider = desiredSteeringWheelProvider;
      this.handComplianceControlParametersProvider = handComplianceControlParametersProvider;
      this.footPoseProvider = footPoseProvider;

      this.automaticManipulationAbortCommunicator = automaticManipulationAbortCommunicator;

      this.desiredHandLoadBearingProvider = desiredHandLoadBearingProvider;
      this.desiredFootLoadBearingProvider = footStateProvider;
      this.desiredThighLoadBearingProvider = thighLoadBearingProvider;
      this.desiredPelvisLoadBearingProvider = pelvisLoadBearingProvider;

      this.controlStatusProducer = controlStatusProducer;

      this.capturabilityBasedStatusProducer = capturabilityBasedStatusProducer;
      this.desiredJointsPositionProvider = desiredJointsPositionProvider;
      this.singleJointPositionProvider = singleJointPositionProvider;
      this.multiJointPositionProvider = multiJointPositionProvider;

      this.handPoseStatusProducer = handPoseStatusProducer;

      this.objectWeightProvider = objectWeightProvider;

      if (abortProvider == null)
      {
         this.abortProvider = new AbortWalkingProvider();
      }
      else
      {
         this.abortProvider = abortProvider;
      }

   }

   public void clearPoseProviders()
   {
      if (desiredPelvisPoseProvider != null)
      {
         desiredPelvisPoseProvider.getDesiredPelvisPosition(ReferenceFrame.getWorldFrame());
         desiredPelvisPoseProvider.getDesiredPelvisOrientation(ReferenceFrame.getWorldFrame());
      }

      if (desiredChestOrientationProvider != null)
      {
         desiredChestOrientationProvider.getDesiredChestOrientation();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (desiredHandPoseProvider != null)
         {
            desiredHandPoseProvider.getDesiredHandPose(robotSide);
         }

         if (footPoseProvider != null)
         {
            footPoseProvider.getDesiredFootPose(robotSide);
         }
      }
   }

   public DesiredHighLevelStateProvider getDesiredHighLevelStateProvider()
   {
      return desiredHighLevelStateProvider;
   }

   public FootstepProvider getFootstepProvider()
   {
      return footstepProvider;
   }

   public HandstepProvider getHandstepProvider()
   {
      return handstepProvider;
   }

   public HeadOrientationProvider getDesiredHeadOrientationProvider()
   {
      return desiredHeadOrientationProvider;
   }

   public DesiredJointsPositionProvider getDesiredJointsPositionProvider()
   {
      return desiredJointsPositionProvider;
   }

   public SingleJointPositionProvider getSingleJointPositionProvider()
   {
      return singleJointPositionProvider;
   }
   
   public MultiJointPositionProvider getMultiJointPositionProvider()
   {
      return multiJointPositionProvider;
   }

   public PelvisPoseProvider getDesiredPelvisPoseProvider()
   {
      return desiredPelvisPoseProvider;
   }

   public DesiredComHeightProvider getDesiredComHeightProvider()
   {
      return desiredComHeightProvider;
   }

   public DesiredPelvisLoadBearingProvider getDesiredPelvisLoadBearingProvider()
   {
      return desiredPelvisLoadBearingProvider;
   }

   public HandPoseProvider getDesiredHandPoseProvider()
   {
      return desiredHandPoseProvider;
   }

   public HandComplianceControlParametersProvider getHandComplianceControlParametersProvider()
   {
      return handComplianceControlParametersProvider;
   }

   public DesiredSteeringWheelProvider getDesiredSteeringWheelProvider()
   {
      return desiredSteeringWheelProvider;
   }

   public HandLoadBearingProvider getDesiredHandLoadBearingProvider()
   {
      return desiredHandLoadBearingProvider;
   }

   public HashMap<Footstep, TrajectoryParameters> getMapFromFootstepsToTrajectoryParameters()
   {
      return mapFromFootstepsToTrajectoryParameters;
   }

   public ChestOrientationProvider getDesiredChestOrientationProvider()
   {
      return desiredChestOrientationProvider;
   }

   public FootPoseProvider getDesiredFootPoseProvider()
   {
      return footPoseProvider;
   }

   public DesiredFootStateProvider getDesiredFootStateProvider()
   {
      return desiredFootLoadBearingProvider;
   }

   public DesiredThighLoadBearingProvider getDesiredThighLoadBearingProvider()
   {
      return desiredThighLoadBearingProvider;
   }


   public ControlStatusProducer getControlStatusProducer()
   {
      return controlStatusProducer;
   }

   public CapturabilityBasedStatusProducer getCapturabilityBasedStatusProducer()
   {
      return capturabilityBasedStatusProducer;
   }

   public HandPoseStatusProducer getHandPoseStatusProducer()
   {
      return handPoseStatusProducer;
   }

   public ObjectWeightProvider getObjectWeightProvider()
   {
      return objectWeightProvider;
   }

   public AbortWalkingProvider getAbortProvider()
   {
      return abortProvider;
   }

   public AutomaticManipulationAbortCommunicator getAutomaticManipulationAbortCommunicator()
   {
      return automaticManipulationAbortCommunicator;
   }
}
