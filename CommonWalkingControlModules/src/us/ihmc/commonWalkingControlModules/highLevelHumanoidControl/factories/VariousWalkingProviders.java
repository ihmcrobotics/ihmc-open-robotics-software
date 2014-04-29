package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.spine.ChestOrientationPacket;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.BlindWalkingPacket;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.PauseCommand;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.packetConsumers.*;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.NetworkControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packets.*;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.terrain.VaryingStairGroundProfile;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;

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
   private final DesiredComHeightProvider desiredComHeightProvider;
   private final DesiredHandPoseProvider desiredHandPoseProvider;
   private final DesiredChestOrientationProvider desiredChestOrientationProvider;
   private final DesiredFootPoseProvider desiredFootPoseProvider;
   private final ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider;

   // TODO: Rename DesiredFootStateProvider to DesiredFootLoadBearingProvider, do the same for the packet, etc.
   private final DesiredFootStateProvider desiredFootLoadBearingProvider;
   private final DesiredHandLoadBearingProvider desiredHandLoadBearingProvider;
   private final DesiredPelvisLoadBearingProvider desiredPelvisLoadBearingProvider;
   private final DesiredThighLoadBearingProvider desiredThighLoadBearingProvider;
   private final DesiredArmJointAngleProvider desiredArmJointAngleProvider;
   
   // TODO: Shouldn't really be in providers but this class is the easiest to access
   private final ControlStatusProducer controlStatusProducer;

   public VariousWalkingProviders(FootstepProvider footstepProvider, HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters,
         DesiredHeadOrientationProvider desiredHeadOrientationProvider, DesiredComHeightProvider desiredComHeightProvider,
         DesiredPelvisPoseProvider desiredPelvisPoseProvider,
         DesiredHandPoseProvider desiredHandPoseProvider, DesiredHandLoadBearingProvider desiredHandLoadBearingProvider,
         DesiredChestOrientationProvider desiredChestOrientationProvider,
         DesiredFootPoseProvider footPoseProvider, DesiredFootStateProvider footStateProvider,
         DesiredHighLevelStateProvider desiredHighLevelStateProvider, DesiredThighLoadBearingProvider thighLoadBearingProvider,
         DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider, DesiredArmJointAngleProvider desiredArmJointAngleProvider,
         ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider, ControlStatusProducer controlStatusProducer)
   {
      this.desiredHighLevelStateProvider = desiredHighLevelStateProvider;
      this.footstepProvider = footstepProvider;
      this.mapFromFootstepsToTrajectoryParameters = mapFromFootstepsToTrajectoryParameters;
      this.desiredHeadOrientationProvider = desiredHeadOrientationProvider;
      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.desiredComHeightProvider = desiredComHeightProvider;
      this.desiredChestOrientationProvider = desiredChestOrientationProvider;
      this.desiredHandPoseProvider = desiredHandPoseProvider;
      this.desiredFootPoseProvider = footPoseProvider;
      this.reinitializeWalkingControllerProvider = reinitializeWalkingControllerProvider;
      
      this.desiredHandLoadBearingProvider = desiredHandLoadBearingProvider;
      this.desiredFootLoadBearingProvider = footStateProvider;
      this.desiredThighLoadBearingProvider = thighLoadBearingProvider;
      this.desiredPelvisLoadBearingProvider = pelvisLoadBearingProvider;
      
      this.desiredArmJointAngleProvider = desiredArmJointAngleProvider;
      
      this.controlStatusProducer = controlStatusProducer;
      
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
   
   public DesiredComHeightProvider getDesiredComHeightProvider()
   {
      return desiredComHeightProvider;
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

   public DesiredArmJointAngleProvider getDesiredArmJointAngleProvider()
   {
      return desiredArmJointAngleProvider;
   }
   
   public ReinitializeWalkingControllerProvider getReinitializeWalkingControllerProvider()
   {
      return reinitializeWalkingControllerProvider;
   }


   public static VariousWalkingProviders createUsingObjectCommunicator(FootstepTimingParameters footstepTimingParameters, GlobalDataProducer objectCommunicator, FullRobotModel fullRobotModel,
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<ContactablePlaneBody> bipedFeet, ConstantTransferTimeCalculator transferTimeCalculator,
           ConstantSwingTimeCalculator swingTimeCalculator, YoVariableRegistry registry)
   {
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters, registry);

      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      BlindWalkingToDestinationDesiredFootstepCalculator desiredFootstepCalculator =
         HighLevelHumanoidControllerFactoryHelper.getBlindWalkingToDestinationDesiredFootstepCalculator(walkingControllerParameters, referenceFrames,
            bipedFeet, registry);

      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, desiredFootstepCalculator, swingTimeCalculator,
                                                           transferTimeCalculator, registry);
      
      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(bipedFeet, footstepPathCoordinator, mapFromFootstepsToTrajectoryParameters);
      BlindWalkingPacketConsumer blindWalkingPacketConsumer = new BlindWalkingPacketConsumer(footstepPathCoordinator);
      PauseCommandConsumer pauseCommandConsumer = new PauseCommandConsumer(footstepPathCoordinator);
      DesiredHighLevelStateProvider highLevelStateProvider = new DesiredHighLevelStateProvider();
      DesiredHeadOrientationProvider headOrientationProvider = new DesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame());
      DesiredComHeightProvider desiredComHeightProvider = new DesiredComHeightProvider();
      DesiredPelvisPoseProvider pelvisPoseProvider = new DesiredPelvisPoseProvider();
      DesiredChestOrientationProvider chestOrientationProvider = new DesiredChestOrientationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = new ReinitializeWalkingControllerProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      
      DesiredArmJointAngleProvider armJointAngleProvider = new DesiredArmJointAngleProvider(fullRobotModel);

      objectCommunicator.attachListener(FootstepDataList.class, footstepPathConsumer);
      objectCommunicator.attachListener(BlindWalkingPacket.class, blindWalkingPacketConsumer);
      objectCommunicator.attachListener(PauseCommand.class, pauseCommandConsumer);
      objectCommunicator.attachListener(HighLevelStatePacket.class, highLevelStateProvider);
      objectCommunicator.attachListener(HeadOrientationPacket.class, headOrientationProvider.getHeadOrientationPacketConsumer());
      objectCommunicator.attachListener(ComHeightPacket.class, desiredComHeightProvider.getComHeightPacketConsumer());
      objectCommunicator.attachListener(LookAtPacket.class, headOrientationProvider.getLookAtPacketConsumer());
      objectCommunicator.attachListener(PelvisOrientationPacket.class, pelvisPoseProvider);
      objectCommunicator.attachListener(HandPosePacket.class, handPoseProvider);
      objectCommunicator.attachListener(FootPosePacket.class, footPoseProvider);
      objectCommunicator.attachListener(ChestOrientationPacket.class, chestOrientationProvider);
      objectCommunicator.attachListener(ReinitializeWalkingControllerPacket.class, reinitializeWalkingControllerProvider);
      
      objectCommunicator.attachListener(ArmJointAnglePacket.class, armJointAngleProvider);

      objectCommunicator.attachListener(HandLoadBearingPacket.class, handLoadBearingProvider);
      objectCommunicator.attachListener(FootStatePacket.class, footLoadBearingProvider);
      objectCommunicator.attachListener(ThighStatePacket.class, thighLoadBearingProvider);
      objectCommunicator.attachListener(BumStatePacket.class, pelvisLoadBearingProvider);

      
      ControlStatusProducer controlStatusProducer = new NetworkControlStatusProducer(objectCommunicator);
      
      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepPathCoordinator, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, desiredComHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider,
            chestOrientationProvider, footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider,
            pelvisLoadBearingProvider, armJointAngleProvider, reinitializeWalkingControllerProvider, controlStatusProducer);

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
      DesiredComHeightProvider comHeightProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredHandPoseProvider handPoseProvider = null;
      if(fullRobotModel.getHand(RobotSide.LEFT) != null && fullRobotModel.getHand(RobotSide.RIGHT) != null)
         handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters, registry);
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = new ReinitializeWalkingControllerProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
      DesiredFootStateProvider footLoadBearingProvider = new DesiredFootStateProvider();
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = new DesiredPelvisLoadBearingProvider();
      DesiredThighLoadBearingProvider thighLoadBearingProvider = new DesiredThighLoadBearingProvider();

      DesiredArmJointAngleProvider armJointAngleProvider = new DesiredArmJointAngleProvider(fullRobotModel);
      
      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();

      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, chestOrientationProvider,
            footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider, armJointAngleProvider,
            reinitializeWalkingControllerProvider, controlStatusProducer);

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
      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      DesiredHighLevelStateProvider highLevelStateProvider = null;
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredComHeightProvider comHeightProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredHandPoseProvider handPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredFootPoseProvider footPoseProvider = null;
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = null;

      DesiredHandLoadBearingProvider handLoadBearingProvider = null;
      DesiredFootStateProvider footLoadBearingProvider = null;
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null;
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null;
      
      DesiredArmJointAngleProvider armJointAngleProvider = null;
      
      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();
      
      
      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, chestOrientationProvider,
            footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider, armJointAngleProvider,
            reinitializeWalkingControllerProvider, controlStatusProducer);

      return variousWalkingProviders;
   }

   public static VariousWalkingProviders createUsingYoVariables(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters,
         CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> bipedFeet, ConstantTransferTimeCalculator transferTimeCalculator,
         ConstantSwingTimeCalculator swingTimeCalculator, YoVariableRegistry registry)
   {
      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters, registry);

      LinkedHashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = new LinkedHashMap<Footstep, TrajectoryParameters>();

      FootstepProvider footstepPovider = new UserDesiredFootstepProvider(bipedFeet, referenceFrames.getAnkleZUpReferenceFrames(), registry);
      
      DesiredHighLevelStateProvider highLevelStateProvider = null; 
      DesiredHeadOrientationProvider headOrientationProvider = new UserDesiredHeadOrientationProvider(referenceFrames.getPelvisZUpFrame(), registry); 
      DesiredComHeightProvider desiredComHeightProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null; 
      DesiredFootPoseProvider footPoseProvider = null; 
      ReinitializeWalkingControllerProvider reinitializeWalkingControllerProvider = null; 

      DesiredHandLoadBearingProvider handLoadBearingProvider = null; 
      DesiredFootStateProvider footLoadBearingProvider = null; 
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null; 
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null; 
      
      DesiredArmJointAngleProvider armJointAngleProvider = null; 


      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();
      
      VariousWalkingProviders variousProvidersFactory = new VariousWalkingProviders(footstepPovider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, desiredComHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, chestOrientationProvider,
            footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider, pelvisLoadBearingProvider, armJointAngleProvider,
            reinitializeWalkingControllerProvider, controlStatusProducer);

      return variousProvidersFactory;
   }

   public ControlStatusProducer getControlStatusProducer()
   {
      return controlStatusProducer;
   }
}
