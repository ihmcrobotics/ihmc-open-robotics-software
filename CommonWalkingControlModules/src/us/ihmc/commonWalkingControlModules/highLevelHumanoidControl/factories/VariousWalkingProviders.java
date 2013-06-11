package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.spine.ChestOrientationPacket;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.PauseCommand;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.packetConsumers.*;
import us.ihmc.commonWalkingControlModules.packets.*;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.terrain.VaryingStairGroundProfile;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectCommunicator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

public class VariousWalkingProviders
{
   private final FootstepProvider footstepProvider;
   private final HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters;

   private final DesiredHighLevelStateProvider desiredHighLevelStateProvider;
   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;
   private final DesiredPelvisPoseProvider desiredPelvisPoseProvider;
   private final DesiredHandPoseProvider desiredHandPoseProvider;
   private final TorusPoseProvider torusPoseProvider;
   private final TorusManipulationProvider torusManipulationProvider;
   private final DesiredChestOrientationProvider desiredChestOrientationProvider;
   private final DesiredFootPoseProvider desiredFootPoseProvider;
   private final PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider;
   private final ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider;

   // TODO: Rename DesiredFootStateProvider to DesiredFootLoadBearingProvider, do the same for the packet, etc.
   private final DesiredFootStateProvider desiredFootLoadBearingProvider;
   private final DesiredHandLoadBearingProvider desiredHandLoadBearingProvider;
   private final DesiredPelvisLoadBearingProvider desiredPelvisLoadBearingProvider;
   private final DesiredThighLoadBearingProvider desiredThighLoadBearingProvider;
   private final FingerStateProvider fingerStateProvider;

   public VariousWalkingProviders(FootstepProvider footstepProvider, HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters,
         DesiredHeadOrientationProvider desiredHeadOrientationProvider, DesiredPelvisPoseProvider desiredPelvisPoseProvider,
         DesiredHandPoseProvider desiredHandPoseProvider, DesiredHandLoadBearingProvider desiredHandLoadBearingProvider, TorusPoseProvider torusPoseProvider,
         TorusManipulationProvider torusManipulationProvider, DesiredChestOrientationProvider desiredChestOrientationProvider,
         DesiredFootPoseProvider footPoseProvider, DesiredFootStateProvider footStateProvider, PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider,
         DesiredHighLevelStateProvider desiredHighLevelStateProvider, DesiredThighLoadBearingProvider thighLoadBearingProvider,
         DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider, FingerStateProvider fingerStateProvider,
         ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider)
   {
      this.desiredHighLevelStateProvider = desiredHighLevelStateProvider;
      this.footstepProvider = footstepProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.desiredChestOrientationProvider = desiredChestOrientationProvider;
      this.torusPoseProvider = torusPoseProvider;
      this.torusManipulationProvider = torusManipulationProvider;
      this.desiredHandPoseProvider = desiredHandPoseProvider;
      this.desiredFootPoseProvider = footPoseProvider;
      this.pelvisPoseWithRespectToVehicleProvider = pelvisPoseWithRespectToVehicleProvider;
      this.reinitializeWalkingControllerProvider = reinitializeWalkingControllerProvider;
      
      this.desiredHandLoadBearingProvider = desiredHandLoadBearingProvider;
      this.desiredFootLoadBearingProvider = footStateProvider;
      this.desiredThighLoadBearingProvider = thighLoadBearingProvider;
      this.desiredPelvisLoadBearingProvider = pelvisLoadBearingProvider;
      this.fingerStateProvider = fingerStateProvider;
   }
   
   public void clearPoseProviders()
   {
      if(desiredPelvisPoseProvider != null)
      {
         desiredPelvisPoseProvider.getDesiredPelvisPose();
      }
      if(desiredChestOrientationProvider != null)
      {
         desiredChestOrientationProvider.getDesiredChestOrientation();
      }
      
      for(RobotSide robotSide : RobotSide.values)
      {
         if(desiredHandPoseProvider != null)
         {
            desiredHandPoseProvider.getDesiredHandPose(robotSide);
         }
         
         if(desiredFootPoseProvider != null)
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

   public DesiredHeadOrientationProvider getDesiredHeadOrientationProvider()
   {
      return desiredHeadOrientationProvider;
   }

   public DesiredPelvisPoseProvider getDesiredPelvisPoseProvider()
   {
      return desiredPelvisPoseProvider;
   }

   public DesiredPelvisLoadBearingProvider getDesiredPelvisLoadBearingProvider()
   {
      return desiredPelvisLoadBearingProvider;
   }
   
   public DesiredHandPoseProvider getDesiredHandPoseProvider()
   {
      return desiredHandPoseProvider;
   }

   public DesiredHandLoadBearingProvider getDesiredHandLoadBearingProvider()
   {
      return desiredHandLoadBearingProvider;
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

   public DesiredFootStateProvider getDesiredFootStateProvider()
   {
      return desiredFootLoadBearingProvider;
   }
   
   public DesiredThighLoadBearingProvider getDesiredThighLoadBearingProvider()
   {
      return desiredThighLoadBearingProvider;
   }

   public PelvisPoseWithRespectToVehicleProvider getPelvisPoseWithRespectToVehicleProvider()
   {
      return pelvisPoseWithRespectToVehicleProvider;
   }

   public FingerStateProvider getFingerStateProvider()
   {
      return fingerStateProvider;
   }
   
   public ReinitializeWalkingControllerProvider getReinitializeWalkingControllerProvider()
   {
      return reinitializeWalkingControllerProvider;
   }

   public static VariousWalkingProviders createUsingObjectCommunicator(ObjectCommunicator objectCommunicator, FullRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<ContactablePlaneBody> bipedFeet, ConstantTransferTimeCalculator transferTimeCalculator,
           ConstantSwingTimeCalculator swingTimeCalculator, YoVariableRegistry registry)
   {
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
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
      DesiredHighLevelStateProvider highLevelStateProvider = new DesiredHighLevelStateProvider();
      DesiredHeadOrientationProvider headOrientationProvider = new DesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame());
      DesiredPelvisPoseProvider pelvisPoseProvider = new DesiredPelvisPoseProvider();
      DesiredChestOrientationProvider chestOrientationProvider = new DesiredChestOrientationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();
      PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider = new PelvisPoseWithRespectToVehicleProvider();
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = new ReinitializeWalkingControllerProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      FingerStateProvider fingerStateProvider = new FingerStateProvider();


      objectCommunicator.attachListener(FootstepDataList.class, footstepPathConsumer);
      objectCommunicator.attachListener(BlindWalkingPacket.class, blindWalkingPacketConsumer);
      objectCommunicator.attachListener(PauseCommand.class, pauseCommandConsumer);
      objectCommunicator.attachListener(HighLevelStatePacket.class, highLevelStateProvider);
      objectCommunicator.attachListener(HeadOrientationPacket.class, headOrientationProvider.getHeadOrientationPacketConsumer());
      objectCommunicator.attachListener(LookAtPacket.class, headOrientationProvider.getLookAtPacketConsumer());
      objectCommunicator.attachListener(PelvisOrientationPacket.class, pelvisPoseProvider);
      objectCommunicator.attachListener(HandPosePacket.class, handPoseProvider);
      objectCommunicator.attachListener(TorusPosePacket.class, torusPoseProvider);
      objectCommunicator.attachListener(TorusManipulationPacket.class, torusManipulationProvider);
      objectCommunicator.attachListener(FootPosePacket.class, footPoseProvider);
      objectCommunicator.attachListener(ChestOrientationPacket.class, chestOrientationProvider);
      objectCommunicator.attachListener(PelvisPoseWithRespectToVehiclePacket.class, pelvisPoseWithRespectToVehicleProvider);
      objectCommunicator.attachListener(ReinitializeWalkingControllerPacket.class, reinitializeWalkingControllerProvider);

      objectCommunicator.attachListener(HandLoadBearingPacket.class, handLoadBearingProvider);
      objectCommunicator.attachListener(FootStatePacket.class, footLoadBearingProvider);
      objectCommunicator.attachListener(ThighStatePacket.class, thighLoadBearingProvider);
      objectCommunicator.attachListener(BumStatePacket.class, pelvisLoadBearingProvider);

      objectCommunicator.attachListener(HandStatePacket.class, fingerStateProvider);

      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepPathCoordinator, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, torusPoseProvider, torusManipulationProvider,
            chestOrientationProvider, footPoseProvider, footLoadBearingProvider, pelvisPoseWithRespectToVehicleProvider, highLevelStateProvider, thighLoadBearingProvider,
            pelvisLoadBearingProvider, fingerStateProvider, reinitializeWalkingControllerProvider);

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

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();
      PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider = new PelvisPoseWithRespectToVehicleProvider();
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = new ReinitializeWalkingControllerProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();
      FingerStateProvider fingerStateProvider = new FingerStateProvider();

      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           headOrientationProvider, pelvisPoseProvider, handPoseProvider,
                                                           handLoadBearingProvider, torusPoseProvider, torusManipulationProvider,
                                                           chestOrientationProvider, footPoseProvider, footLoadBearingProvider,
            pelvisPoseWithRespectToVehicleProvider, highLevelStateProvider, thighLoadBearingProvider,
                                                           pelvisLoadBearingProvider, fingerStateProvider, reinitializeWalkingControllerProvider);

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
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredHandPoseProvider handPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredFootPoseProvider footPoseProvider = null;
      PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider = null;
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = null;

      DesiredHandLoadBearingProvider handLoadBearingProvider = null;
      DesiredFootStateProvider footLoadBearingProvider = null;
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null;
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null;
      FingerStateProvider fingerStateProvider = null;
      
      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           headOrientationProvider, pelvisPoseProvider, handPoseProvider,
                                                           handLoadBearingProvider, torusPoseProvider, torusManipulationProvider,
                                                           chestOrientationProvider, footPoseProvider, footLoadBearingProvider,
            pelvisPoseWithRespectToVehicleProvider, highLevelStateProvider, thighLoadBearingProvider,
                                                           pelvisLoadBearingProvider, fingerStateProvider, reinitializeWalkingControllerProvider);

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
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();


      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredFootPoseProvider footPoseProvider = null;
      PelvisPoseWithRespectToVehicleProvider pelvisPoseWithRespectToVehicleProvider = null;
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = null;

      DesiredHandLoadBearingProvider handLoadBearingProvider = null;
      DesiredFootStateProvider footLoadBearingProvider = null;
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null;
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null;

      FingerStateProvider fingerStateProvider = null;
      
      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
                                                           headOrientationProvider, pelvisPoseProvider, handPoseProvider,
                                                           handLoadBearingProvider, torusPoseProvider, torusManipulationProvider,
                                                           chestOrientationProvider, footPoseProvider, footLoadBearingProvider,
            pelvisPoseWithRespectToVehicleProvider, highLevelStateProvider, thighLoadBearingProvider,
                                                           pelvisLoadBearingProvider, fingerStateProvider, reinitializeWalkingControllerProvider);

      return variousWalkingProviders;
   }
}
