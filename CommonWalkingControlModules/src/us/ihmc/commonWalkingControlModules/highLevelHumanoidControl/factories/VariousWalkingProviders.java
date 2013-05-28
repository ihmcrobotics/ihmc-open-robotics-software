package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.desiredChestOrientation.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationPacket;
import us.ihmc.commonWalkingControlModules.controlModules.head.LookAtPacket;
import us.ihmc.commonWalkingControlModules.controlModules.head.PelvisOrientationPacket;
import us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.controlModules.spine.ChestOrientationPacket;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BlindWalkingPacketConsumer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BlindWalkingToDestinationDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BoxDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorFootstepProviderWrapper;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepPathConsumer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepPathCoordinator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.desiredFootStep.PauseCommandConsumer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.VaryingStairDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.PauseCommand;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.FootPosePacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.HandPosePacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusManipulationProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusPosePacket;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.TorusPoseProvider;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.terrain.VaryingStairGroundProfile;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;

public class VariousWalkingProviders
{
   private final FootstepProvider footstepProvider;
   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;
   private final DesiredPelvisPoseProvider desiredPelvisPoseProvider;
   private final DesiredHandPoseProvider desiredHandPoseProvider;
   private final TorusPoseProvider torusPoseProvider;
   private final TorusManipulationProvider torusManipulationProvider;
   private final DesiredChestOrientationProvider desiredChestOrientationProvider;
   private final DesiredFootPoseProvider desiredFootPoseProvider;

   public VariousWalkingProviders(FootstepProvider footstepProvider, HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters,
                                  DesiredHeadOrientationProvider desiredHeadOrientationProvider, DesiredPelvisPoseProvider desiredPelvisPoseProvider,
                                  DesiredHandPoseProvider desiredHandPoseProvider, TorusPoseProvider torusPoseProvider,
                                  TorusManipulationProvider torusManipulationProvider, DesiredChestOrientationProvider desiredChestOrientationProvider,
                                  DesiredFootPoseProvider footPoseProvider)
   {
      this.footstepProvider = footstepProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;

      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.desiredHandPoseProvider = desiredHandPoseProvider;
      this.torusPoseProvider = torusPoseProvider;
      this.torusManipulationProvider = torusManipulationProvider;

      this.desiredChestOrientationProvider = desiredChestOrientationProvider;
      this.desiredFootPoseProvider = footPoseProvider;
   }

   public FootstepProvider getFootstepProvider()
   {
      return footstepProvider;
   }

   public DesiredHeadOrientationProvider getDesiredHeadOrientationProvider()
   {
      return desiredHeadOrientationProvider;
   }

   public DesiredPelvisPoseProvider getDesiredPelvisPoseProvider()
   {
      return desiredPelvisPoseProvider;
   }

   public DesiredHandPoseProvider getDesiredHandPoseProvider()
   {
      return desiredHandPoseProvider;
   }

   public TorusManipulationProvider getTorusManipulationProvider()
   {
      return torusManipulationProvider;
   }

   public TorusPoseProvider getTorusPoseProvider()
   {
      return torusPoseProvider;
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

   public static VariousWalkingProviders createUsingObjectCommunicator(ObjectCommunicator objectCommunicator, FullRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<ContactablePlaneBody> bipedFeet, ConstantTransferTimeCalculator transferTimeCalculator,
           ConstantSwingTimeCalculator swingTimeCalculator, YoVariableRegistry registry)
   {
      DesiredHandPoseProvider desiredHandPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();

      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      BlindWalkingToDestinationDesiredFootstepCalculator desiredFootstepCalculator =
         HighLevelHumanoidControllerFactoryHelper.getBlindWalkingToDestinationDesiredFootstepCalculator(walkingControllerParameters, referenceFrames,
            bipedFeet, registry);

      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(objectCommunicator, desiredFootstepCalculator, swingTimeCalculator,
                                                           transferTimeCalculator, registry);
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(bipedFeet, footstepPathCoordinator, mapFromFootstepsToTrajectoryParameters);
      BlindWalkingPacketConsumer blindWalkingPacketConsumer = new BlindWalkingPacketConsumer(footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(footstepPathCoordinator);
      DesiredHeadOrientationProvider desiredHeadOrientationProvider = new DesiredHeadOrientationProvider();
      DesiredPelvisPoseProvider desiredPelvisPoseProvider = new DesiredPelvisPoseProvider();
      DesiredChestOrientationProvider desiredChestOrientationProvider = new DesiredChestOrientationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();


      objectCommunicator.attachListener(FootstepDataList.class, footstepPathConsumer);
      objectCommunicator.attachListener(BlindWalkingPacket.class, blindWalkingPacketConsumer);
      objectCommunicator.attachListener(PauseCommand.class, pauseCommandConsumer);
      objectCommunicator.attachListener(HeadOrientationPacket.class, desiredHeadOrientationProvider.getHeadOrientationPacketConsumer());
      objectCommunicator.attachListener(LookAtPacket.class, desiredHeadOrientationProvider.getLookAtPacketConsumer());
      objectCommunicator.attachListener(PelvisOrientationPacket.class, desiredPelvisPoseProvider);
      objectCommunicator.attachListener(HandPosePacket.class, desiredHandPoseProvider);
      objectCommunicator.attachListener(TorusPosePacket.class, torusPoseProvider);
      objectCommunicator.attachListener(FootPosePacket.class, footPoseProvider);
      objectCommunicator.attachListener(ChestOrientationPacket.class, desiredChestOrientationProvider);


      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepPathCoordinator, mapFromFootstepsToTrajectoryParameters,
                                                           desiredHeadOrientationProvider, desiredPelvisPoseProvider, desiredHandPoseProvider,
                                                           torusPoseProvider, torusManipulationProvider, desiredChestOrientationProvider, footPoseProvider);

      return variousProvidersFactory;
   }

   public static VariousWalkingProviders createUsingComponentBasedDesiredFootstepCalculator(FullRobotModel fullRobotModel,
           CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> bipedFeet, double controlDT, ArrayList<Updatable> updatables,
           boolean useHeadingAndVelocityScript, GroundProfile groundProfileForCheatingOnStepHeight, WalkingControllerParameters walkingControllerParameters,
           YoVariableRegistry registry)
   {
      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator =
         HighLevelHumanoidControllerFactoryHelper.getDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, bipedFeet, controlDT, registry,
            updatables, useHeadingAndVelocityScript);
      if (groundProfileForCheatingOnStepHeight != null)
      {
         desiredFootstepCalculator.setGroundProfile(groundProfileForCheatingOnStepHeight);
      }

      DesiredFootstepCalculatorFootstepProviderWrapper footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator,
                                                                             registry);
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHeadOrientationProvider desiredHeadOrientationProvider = null;
      DesiredPelvisPoseProvider desiredPelvisPoseProvider = null;
      DesiredChestOrientationProvider desiredChestOrientationProvider = null;
      DesiredHandPoseProvider desiredHandPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      DesiredFootPoseProvider desiredFootPoseProvider = new DesiredFootPoseProvider();

      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           desiredHeadOrientationProvider, desiredPelvisPoseProvider, desiredHandPoseProvider,
                                                           torusPoseProvider, torusManipulationProvider, desiredChestOrientationProvider,
                                                           desiredFootPoseProvider);

      return variousProvidersFactory;

   }

   public static VariousWalkingProviders createForStairClimbing(WalkingControllerParameters walkingControllerParameters,
           VaryingStairGroundProfile groundProfile, SideDependentList<ContactablePlaneBody> bipedFeet, CommonWalkingReferenceFrames referenceFrames,
           DesiredHeadingControlModule desiredHeadingControlModule, FullRobotModel fullRobotModel, YoVariableRegistry registry)
   {
      VaryingStairDesiredFootstepCalculator desiredFootstepCalculator = new VaryingStairDesiredFootstepCalculator(groundProfile, bipedFeet, referenceFrames,
                                                                           desiredHeadingControlModule, registry);
      desiredFootstepCalculator.setupParametersForR2InverseDynamics();
      FootstepProvider footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator, registry);
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHeadOrientationProvider desiredHeadOrientationProvider = null;
      DesiredPelvisPoseProvider desiredPelvisPoseProvider = null;
      DesiredHandPoseProvider desiredHandPoseProvider = null;
      DesiredChestOrientationProvider desiredChestOrientationProvider = null;
      DesiredFootPoseProvider desiredFootPoseProvider = null;

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           desiredHeadOrientationProvider, desiredPelvisPoseProvider, desiredHandPoseProvider,
                                                           torusPoseProvider, torusManipulationProvider, desiredChestOrientationProvider,
                                                           desiredFootPoseProvider);

      return variousWalkingProviders;
   }

   public static VariousWalkingProviders createForBoxStepping(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters,
           SideDependentList<? extends ContactablePlaneBody> bipedFeet, CommonWalkingReferenceFrames referenceFrames, double controlDT,
           YoVariableRegistry registry, double leadingFootPitch, double boxLength, double boxWidth, double closeStanceWidth)
   {
      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
      desiredHeadingControlModule.setMaxHeadingDot(0.4);

      BoxDesiredFootstepCalculator desiredFootstepCalculator = new BoxDesiredFootstepCalculator(bipedFeet, referenceFrames.getAnkleZUpReferenceFrames(),
                                                                  desiredHeadingControlModule, registry, leadingFootPitch);
      desiredFootstepCalculator.setBoxLength(boxLength);
      desiredFootstepCalculator.setBoxWidth(boxWidth);
      desiredFootstepCalculator.setCloseStanceWidth(closeStanceWidth);

      FootstepProvider footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator, registry);
      DesiredHeadOrientationProvider desiredHeadOrientationProvider = null;
      DesiredHandPoseProvider desiredHandPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();


      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();
      DesiredPelvisPoseProvider desiredPelvisPoseProvider = null;

      DesiredChestOrientationProvider desiredChestOrientationProvider = null;
      DesiredFootPoseProvider desiredFootPoseProvider = null;

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           desiredHeadOrientationProvider, desiredPelvisPoseProvider, desiredHandPoseProvider,
                                                           torusPoseProvider, torusManipulationProvider, desiredChestOrientationProvider,
                                                           desiredFootPoseProvider);

      return variousWalkingProviders;
   }


}
