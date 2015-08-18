package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorFootstepProviderWrapper;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredSteeringWheelProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.MultiJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectWeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.SingleJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class ComponentBasedVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{
   private final boolean useHeadingAndVelocityScript;
   private final HeightMap heightMapForCheatingOnStepHeight;
   private final double controlDT;

   public ComponentBasedVariousWalkingProviderFactory(boolean useHeadingAndVelocityScript, HeightMap heightMapForCheatingOnStepHeight, double controlDT)
   {
      this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
      this.heightMapForCheatingOnStepHeight = heightMapForCheatingOnStepHeight;
      this.controlDT = controlDT;
   }

   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullHumanoidRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
           ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
           YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      HandstepProvider handstepProvider = null;

      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator =
         HighLevelHumanoidControllerFactoryHelper.getDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, feet, controlDT, registry,
            updatables, useHeadingAndVelocityScript);
      if (heightMapForCheatingOnStepHeight != null)
      {
         desiredFootstepCalculator.setGroundProfile(heightMapForCheatingOnStepHeight);
      }

      DesiredFootstepCalculatorFootstepProviderWrapper footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator,
                                                                             registry);
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      HeadOrientationProvider headOrientationProvider = null;
      DesiredComHeightProvider comHeightProvider = null;
      PelvisPoseProvider pelvisPoseProvider = new UserDesiredPelvisPoseProvider(registry);
      ChestOrientationProvider chestOrientationProvider = null;
      DesiredHandPoseProvider handPoseProvider = null;
      if ((fullRobotModel.getHand(RobotSide.LEFT) != null) && (fullRobotModel.getHand(RobotSide.RIGHT) != null))
         handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame(), null);
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider(walkingControllerParameters.getDefaultSwingTime(), null);
     

      HandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();

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

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, handstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider,
                                                           handComplianceControlParametersProvider, desiredSteeringWheelProvider, handLoadBearingProvider, automaticManipulationAbortCommunicator, chestOrientationProvider, footPoseProvider,
                                                           footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider,
                                                           controlStatusProducer, capturabilityBasedStatusProducer, handPoseStatusProducer, objectWeightProvider,
                                                           desiredJointsPositionProvider, singleJointPositionProvider, abortWalkingProvider, multiJointPositionProvider);

      return variousWalkingProviders;
   }

}
