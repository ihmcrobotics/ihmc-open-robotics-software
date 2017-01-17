package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class DRCFlatGroundWalkingTrack
{
   // looking for CREATE_YOVARIABLE_WALKING_PROVIDERS ?  use the second constructor and pass in WalkingProvider = YOVARIABLE_PROVIDER

   private final AvatarSimulation avatarSimulation;
   
   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model, 
            WalkingProvider.VELOCITY_HEADING_COMPONENT, new HeadingAndVelocityEvaluationScriptParameters()); // should always be committed as VELOCITY_HEADING_COMPONENT
   }

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model, WalkingProvider walkingProvider, HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

      //    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      RobotContactPointParameters contactPointParameters = model.getContactPointParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      ICPOptimizationParameters icpOptimizationParameters = model.getICPOptimizationParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters,
             HighLevelState.WALKING);
      controllerFactory.setICPOptimizationControllerParameters(icpOptimizationParameters);
      controllerFactory.setHeadingAndVelocityEvaluationScriptParameters(walkingScriptParameters);
      

      
      HeightMap heightMapForFootstepZ = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForFootstepZ = scsInitialSetup.getHeightMap();
      }
      
      controllerFactory.createComponentBasedFootstepDataMessageGenerator(useVelocityAndHeadingScript, heightMapForFootstepZ);

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(model);
      avatarSimulationFactory.setMomentumBasedControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(null);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setHumanoidGlobalDataProducer(null);
      avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      avatarSimulation.start();
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      avatarSimulation.getMomentumBasedControllerFactory().attachControllerFailureListener(listener);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return avatarSimulation.getSimulationConstructionSet();
   }

   public AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public void destroySimulation()
   {
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
      }
      GlobalTimer.clearTimers();
   }
}
