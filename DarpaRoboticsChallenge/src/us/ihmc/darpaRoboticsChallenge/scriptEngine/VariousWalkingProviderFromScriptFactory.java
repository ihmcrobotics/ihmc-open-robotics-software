package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.packetConsumers.AutomaticManipulationAbortCommunicator;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandstepProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredSteeringWheelProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.MultiJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ObjectWeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.SingleJointPositionProvider;
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
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


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
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
         ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, 
         YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {
      ScriptBasedFootstepProvider footstepProvider = new ScriptBasedFootstepProvider(scriptFileLoader, time, feet, fullRobotModel, walkingControllerParameters, registry);

      updatables.add(footstepProvider);
      
      DesiredHandPoseProvider handPoseProvider = footstepProvider.getDesiredHandPoseProvider();
      DesiredHandstepProvider handstepProvider = footstepProvider.getDesiredHandstepProvider();
      
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();
      DesiredHighLevelStateProvider highLevelStateProvider = null;
      HeadOrientationProvider headOrientationProvider = new UserDesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame(), registry);
      DesiredComHeightProvider desiredComHeightProvider = footstepProvider.getDesiredComHeightProvider();
      PelvisPoseProvider pelvisPoseProvider = footstepProvider.getDesiredPelvisPoseProvider();
      ChestOrientationProvider chestOrientationProvider = null;
      DesiredFootPoseProvider footPoseProvider = footstepProvider.getDesiredFootPoseProvider();
  
      HandLoadBearingProvider handLoadBearingProvider = null;
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

      VariousWalkingProviders variousProviders = new VariousWalkingProviders(footstepProvider, handstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, desiredComHeightProvider, pelvisPoseProvider, handPoseProvider, handComplianceControlParametersProvider, desiredSteeringWheelProvider, handLoadBearingProvider, automaticManipulationAbortCommunicator, chestOrientationProvider,
            footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider,
            controlStatusProducer, capturabilityBasedStatusProducer, handPoseStatusProducer, objectWeightProvider, desiredJointsPositionProvider,
            singleJointPositionProvider, abortWalkingProvider, multiJointPositionProvider);

      return variousProviders;
   }
}
