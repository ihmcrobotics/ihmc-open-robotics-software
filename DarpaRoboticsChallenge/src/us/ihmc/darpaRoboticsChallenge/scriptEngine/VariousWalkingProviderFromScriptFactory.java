package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmDesiredAccelerationsMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.ArmTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
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
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class VariousWalkingProviderFromScriptFactory implements VariousWalkingProviderFactory
{
   private final ScriptFileLoader scriptFileLoader;

   public VariousWalkingProviderFromScriptFactory(String filename)
   {
      try
      {
         this.scriptFileLoader = new ScriptFileLoader(filename);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Could not load script file " + filename);
      }
   }

   public VariousWalkingProviderFromScriptFactory(InputStream scriptInputStream)
   {
      try
      {
         this.scriptFileLoader = new ScriptFileLoader(scriptInputStream);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         throw new RuntimeException("Could not load script input stream");
      }
   }

   public VariousWalkingProviders createVariousWalkingProviders(final DoubleYoVariable time, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<? extends ContactablePlaneBody> feet,
         ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      ScriptBasedFootstepProvider footstepProvider = new ScriptBasedFootstepProvider(referenceFrames, scriptFileLoader, time, feet, fullRobotModel,
            walkingControllerParameters, registry);

      updatables.add(footstepProvider);

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
      HeadOrientationProvider headOrientationProvider = new UserDesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame(),
            walkingControllerParameters.getTrajectoryTimeHeadOrientation(), registry);
      DesiredComHeightProvider desiredComHeightProvider = footstepProvider.getDesiredComHeightProvider();
      PelvisPoseProvider pelvisPoseProvider = footstepProvider.getDesiredPelvisPoseProvider();
      ChestOrientationProvider chestOrientationProvider = null;

      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      HandPoseStatusProducer handPoseStatusProducer = null;

      AbortWalkingProvider abortWalkingProvider = null;

      HandComplianceControlParametersSubscriber handComplianceControlParametersProvider = null;

      AutomaticManipulationAbortCommunicator automaticManipulationAbortCommunicator = null;

      VariousWalkingProviders variousProviders = new VariousWalkingProviders(handTrajectoryMessageSubscriber, armTrajectoryMessageSubscriber,
            armDesiredAccelerationsMessageSubscriber, headTrajectoryMessageSubscriber, chestTrajectoryMessageSubscriber, pelvisTrajectoryMessageSubscriber,
            footTrajectoryMessageSubscriber, endEffectorLoadBearingMessageSubscriber, stopAllTrajectoryMessageSubscriber,
            pelvisHeightTrajectoryMessageSubscriber, footstepProvider, headOrientationProvider, desiredComHeightProvider, pelvisPoseProvider,
            handComplianceControlParametersProvider, automaticManipulationAbortCommunicator, chestOrientationProvider, highLevelStateProvider, controlStatusProducer,
            capturabilityBasedStatusProducer, handPoseStatusProducer, abortWalkingProvider);

      return variousProviders;
   }
}
