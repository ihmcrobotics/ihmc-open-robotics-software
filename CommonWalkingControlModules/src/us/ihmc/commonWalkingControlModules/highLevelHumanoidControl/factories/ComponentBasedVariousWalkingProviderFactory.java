package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ObsoleteComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ObsoleteDesiredFootstepCalculatorFootstepProviderWrapper;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;
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
      ObsoleteComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = HighLevelHumanoidControllerFactoryHelper
            .getDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, feet, controlDT, registry, updatables, useHeadingAndVelocityScript);
      if (heightMapForCheatingOnStepHeight != null)
      {
         desiredFootstepCalculator.setGroundProfile(heightMapForCheatingOnStepHeight);
      }

      ObsoleteDesiredFootstepCalculatorFootstepProviderWrapper footstepProvider = new ObsoleteDesiredFootstepCalculatorFootstepProviderWrapper(
            desiredFootstepCalculator, registry);
      PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber = null;
      FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber = null;
      StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber = null;

      HighLevelStateMessageSubscriber highLevelStateProvider = null;

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = null;

      AbortWalkingMessageSubscriber abortWalkingProvider = null;

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(pelvisTrajectoryMessageSubscriber, footTrajectoryMessageSubscriber,
            stopAllTrajectoryMessageSubscriber, footstepProvider, highLevelStateProvider, capturabilityBasedStatusProducer,
            abortWalkingProvider);

      return variousWalkingProviders;
   }

}
