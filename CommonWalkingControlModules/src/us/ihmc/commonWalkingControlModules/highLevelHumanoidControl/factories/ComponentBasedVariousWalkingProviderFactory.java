package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorFootstepProviderWrapper;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandstepProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

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

   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> feet,
           ConstantTransferTimeCalculator transferTimeCalculator, ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables,
           YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
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
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredComHeightProvider comHeightProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredHandPoseProvider handPoseProvider = null;
      if ((fullRobotModel.getHand(RobotSide.LEFT) != null) && (fullRobotModel.getHand(RobotSide.RIGHT) != null))
         handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame());
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();
     

      HandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();

      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, handstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider,
                                                           handLoadBearingProvider, chestOrientationProvider, footPoseProvider, footLoadBearingProvider,
                                                           highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider,
                                                           controlStatusProducer);

      return variousWalkingProviders;
   }

}
