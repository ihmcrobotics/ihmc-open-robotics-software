package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.UserDesiredFootstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.EndEffectorLoadBearingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.GoHomeMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisHeightTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisOrientationTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class YoVariableVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{
   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> feet, ConstantTransferTimeCalculator transferTimeCalculator,
         ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      HeadTrajectoryMessageSubscriber headTrajectoryMessageSubscriber = null;
      ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber = null;
      PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber = null;
      PelvisOrientationTrajectoryMessageSubscriber pelvisOrientationTrajectoryMessageSubscriber = null;
      FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber = null;
      EndEffectorLoadBearingMessageSubscriber endEffectorLoadBearingMessageSubscriber = null;
      StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber = null;
      PelvisHeightTrajectoryMessageSubscriber pelvisHeightTrajectoryMessageSubscriber = null;
      GoHomeMessageSubscriber goHomeMessageSubscriber = null;

      FootstepProvider footstepPovider = new UserDesiredFootstepProvider(feet, referenceFrames.getAnkleZUpReferenceFrames(), walkingControllerParameters,
            registry);

      HighLevelStateMessageSubscriber highLevelStateProvider = null;

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      AbortWalkingMessageSubscriber abortWalkingProvider = null;
      HandComplianceControlParametersSubscriber handComplianceControlParametersProvider = null;

      AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator = null;

      VariousWalkingProviders variousProviders = new VariousWalkingProviders(headTrajectoryMessageSubscriber,
            chestTrajectoryMessageSubscriber, pelvisTrajectoryMessageSubscriber, pelvisOrientationTrajectoryMessageSubscriber, footTrajectoryMessageSubscriber,
            endEffectorLoadBearingMessageSubscriber, stopAllTrajectoryMessageSubscriber, pelvisHeightTrajectoryMessageSubscriber,
            goHomeMessageSubscriber, footstepPovider, handComplianceControlParametersProvider, automaticManipulationAbortCommunicator, highLevelStateProvider,
            capturabilityBasedStatusProducer, abortWalkingProvider);

      return variousProviders;
   }

}
