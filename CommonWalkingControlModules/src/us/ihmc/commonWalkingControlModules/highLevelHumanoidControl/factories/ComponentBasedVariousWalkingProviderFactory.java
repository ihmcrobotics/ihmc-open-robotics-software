package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorFootstepProviderWrapper;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmDesiredAccelerationsMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisHeightTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

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
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> feet, ConstantTransferTimeCalculator transferTimeCalculator,
         ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = HighLevelHumanoidControllerFactoryHelper
            .getDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, feet, controlDT, registry, updatables, useHeadingAndVelocityScript);
      if (heightMapForCheatingOnStepHeight != null)
      {
         desiredFootstepCalculator.setGroundProfile(heightMapForCheatingOnStepHeight);
      }

      DesiredFootstepCalculatorFootstepProviderWrapper footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator,
            registry);
      HandTrajectoryMessageSubscriber handTrajectoryMessageSubscriber = null;
      ArmTrajectoryMessageSubscriber armTrajectoryMessageSubscriber = null;
      ArmDesiredAccelerationsMessageSubscriber armDesiredAccelerationsMessageSubscriber = null;
      HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber = null;
      ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber = null;
      PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber = null;
      FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber = null;
      EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber = null;
      StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber = null;
      PelvisHeightTrajectoryMessageSubscriber pelvisHeightTrajectoryMessageSubscriber = null;

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      HeadOrientationProvider headOrientationProvider = null;
      DesiredComHeightProvider comHeightProvider = null;
      PelvisPoseProvider pelvisPoseProvider = new UserDesiredPelvisPoseProvider(registry);
      ChestOrientationProvider chestOrientationProvider = null;
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider(walkingControllerParameters.getDefaultSwingTime(), null);

      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();

      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      HandPoseStatusProducer handPoseStatusProducer = null;

      AbortWalkingProvider abortWalkingProvider = null;

      HandComplianceControlParametersSubscriber handComplianceControlParametersProvider = null;

      AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator = null;

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(handTrajectoryMessageSubscriber, armTrajectoryMessageSubscriber,
            armDesiredAccelerationsMessageSubscriber, headTrajectoryMessageSubscriber, chestTrajectoryMessageSubscriber, pelvisTrajectoryMessageSubscriber,
            footTrajectoryMessageSubscriber, endEffectorLoadBearingMessageSubscriber, stopAllTrajectoryMessageSubscriber,
            pelvisHeightTrajectoryMessageSubscriber, footstepProvider, headOrientationProvider, comHeightProvider, pelvisPoseProvider,
            handComplianceControlParametersProvider, automaticManipulationAbortCommunicator, chestOrientationProvider, footPoseProvider,
            footLoadBearingProvider, highLevelStateProvider, controlStatusProducer, capturabilityBasedStatusProducer, handPoseStatusProducer,
            abortWalkingProvider);

      return variousWalkingProviders;
   }

}
