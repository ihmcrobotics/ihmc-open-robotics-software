package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.UserDesiredFootstepProvider;
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
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredHandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;

public class YoVariableVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{
   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
         ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      HandstepProvider handstepProvider = new UserDesiredHandstepProvider(fullRobotModel, registry, yoGraphicsListRegistry);

      HandPoseProvider handPoseProvider = new UserDesiredHandPoseProvider(fullRobotModel,
            walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame(), registry);

      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      FootstepProvider footstepPovider = new UserDesiredFootstepProvider(feet, referenceFrames.getAnkleZUpReferenceFrames(), walkingControllerParameters, registry);

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      HeadOrientationProvider headOrientationProvider = new UserDesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame(), registry);
      DesiredComHeightProvider desiredComHeightProvider = null;
      PelvisPoseProvider pelvisPoseProvider = new UserDesiredPelvisPoseProvider(registry);
      ChestOrientationProvider chestOrientationProvider = new UserDesiredChestOrientationProvider(referenceFrames.getPelvisZUpFrame(), registry);

      FootPoseProvider footPoseProvider = new UserDesiredFootPoseProvider(fullRobotModel, walkingControllerParameters.getDefaultSwingTime(), registry);

      HandLoadBearingProvider handLoadBearingProvider = new UserDesiredHandLoadBearingProvider(registry);
      DesiredFootStateProvider footLoadBearingProvider = null;
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null;
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null;

      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      HandPoseStatusProducer handPoseStatusProducer = null;
      ObjectWeightProvider objectWeightProvider = null;
      
      DesiredJointsPositionProvider desiredJointsPositionProvider = null;
      SingleJointPositionProvider singleJointPositionProvider = null;
      MultiJointPositionProvider multiJointPositionProvider = null;

      AbortWalkingProvider abortWalkingProvider = null;
      HandComplianceControlParametersProvider handComplianceControlParametersProvider = null;

      DesiredSteeringWheelProvider desiredSteeringWheelProvider = null;

      AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator = null;

      VariousWalkingProviders variousProviders = new VariousWalkingProviders(footstepPovider, handstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, desiredComHeightProvider, pelvisPoseProvider, handPoseProvider, handComplianceControlParametersProvider, desiredSteeringWheelProvider, handLoadBearingProvider, automaticManipulationAbortCommunicator, chestOrientationProvider,
            footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider, controlStatusProducer,
            capturabilityBasedStatusProducer, handPoseStatusProducer, objectWeightProvider, desiredJointsPositionProvider, singleJointPositionProvider, abortWalkingProvider,
            multiJointPositionProvider);

      return variousProviders;
   }

}
