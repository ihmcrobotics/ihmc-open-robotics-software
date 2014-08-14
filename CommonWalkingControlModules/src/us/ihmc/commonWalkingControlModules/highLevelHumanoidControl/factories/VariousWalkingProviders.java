package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ReinitializeWalkingControllerProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packets.DesiredHighLevelStateProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;

public class VariousWalkingProviders
{
   private final FootstepProvider footstepProvider;
   private final HandstepProvider handstepProvider;
   
   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   private final DesiredHighLevelStateProvider desiredHighLevelStateProvider;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;
   private final DesiredPelvisPoseProvider desiredPelvisPoseProvider;
   private final DesiredComHeightProvider desiredComHeightProvider;
   private final HandPoseProvider desiredHandPoseProvider;
   private final DesiredChestOrientationProvider desiredChestOrientationProvider;
   private final DesiredFootPoseProvider desiredFootPoseProvider;
   private final ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider;

   // TODO: Rename DesiredFootStateProvider to DesiredFootLoadBearingProvider, do the same for the packet, etc.
   private final DesiredFootStateProvider desiredFootLoadBearingProvider;
   private final HandLoadBearingProvider desiredHandLoadBearingProvider;
   private final DesiredPelvisLoadBearingProvider desiredPelvisLoadBearingProvider;
   private final DesiredThighLoadBearingProvider desiredThighLoadBearingProvider;

   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final ControlStatusProducer controlStatusProducer;

   public VariousWalkingProviders(FootstepProvider footstepProvider, HandstepProvider handstepProvider, 
         HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters,
         DesiredHeadOrientationProvider desiredHeadOrientationProvider, DesiredComHeightProvider desiredComHeightProvider,
         DesiredPelvisPoseProvider desiredPelvisPoseProvider, HandPoseProvider desiredHandPoseProvider,
         HandLoadBearingProvider desiredHandLoadBearingProvider, DesiredChestOrientationProvider desiredChestOrientationProvider,
         DesiredFootPoseProvider footPoseProvider, DesiredFootStateProvider footStateProvider, DesiredHighLevelStateProvider desiredHighLevelStateProvider,
         DesiredThighLoadBearingProvider thighLoadBearingProvider, DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider,
         ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider, ControlStatusProducer controlStatusProducer)
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
      this.desiredFootPoseProvider = footPoseProvider;
      this.reinitializeWalkingControllerProvider = reinitializeWalkingControllerProvider;

      this.desiredHandLoadBearingProvider = desiredHandLoadBearingProvider;
      this.desiredFootLoadBearingProvider = footStateProvider;
      this.desiredThighLoadBearingProvider = thighLoadBearingProvider;
      this.desiredPelvisLoadBearingProvider = pelvisLoadBearingProvider;

      this.controlStatusProducer = controlStatusProducer;
   }

   public void clearPoseProviders()
   {
      if (desiredPelvisPoseProvider != null)
      {
         desiredPelvisPoseProvider.getDesiredPelvisPose();
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

         if (desiredFootPoseProvider != null)
         {
            desiredFootPoseProvider.getDesiredFootPose(robotSide);
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

   public DesiredHeadOrientationProvider getDesiredHeadOrientationProvider()
   {
      return desiredHeadOrientationProvider;
   }

   public DesiredPelvisPoseProvider getDesiredPelvisPoseProvider()
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

   public HandLoadBearingProvider getDesiredHandLoadBearingProvider()
   {
      return desiredHandLoadBearingProvider;
   }

   public HashMap<Footstep, TrajectoryParameters> getMapFromFootstepsToTrajectoryParameters()
   {
      return mapFromFootstepsToTrajectoryParameters;
   }

   public DesiredChestOrientationProvider getDesiredChestOrientationProvider()
   {
      return desiredChestOrientationProvider;
   }

   public DesiredFootPoseProvider getDesiredFootPoseProvider()
   {
      return desiredFootPoseProvider;
   }

   public DesiredFootStateProvider getDesiredFootStateProvider()
   {
      return desiredFootLoadBearingProvider;
   }

   public DesiredThighLoadBearingProvider getDesiredThighLoadBearingProvider()
   {
      return desiredThighLoadBearingProvider;
   }

   public ReinitializeWalkingControllerProvider getReinitializeWalkingControllerProvider()
   {
      return reinitializeWalkingControllerProvider;
   }

   public ControlStatusProducer getControlStatusProducer()
   {
      return controlStatusProducer;
   }
}
