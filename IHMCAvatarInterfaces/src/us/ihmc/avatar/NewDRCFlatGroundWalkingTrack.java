package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.RobotContactPointParameters;


import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates.*;


public abstract class NewDRCFlatGroundWalkingTrack
{
   // looking for CREATE_YOVARIABLE_WALKING_PROVIDERS ?  use the second constructor and pass in WalkingProvider = YOVARIABLE_PROVIDER

   private final AvatarSimulation avatarSimulation;

   public NewDRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
                                       boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model,
            WalkingProvider.VELOCITY_HEADING_COMPONENT, new HeadingAndVelocityEvaluationScriptParameters()); // should always be committed as VELOCITY_HEADING_COMPONENT
   }

   public NewDRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
                                       boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model, WalkingProvider walkingProvider, HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      //    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      HighLevelControllerParameters highLevelControllerParameters = model.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      RobotContactPointParameters contactPointParameters = model.getContactPointParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames, feetContactSensorNames,
                                                                                                    wristForceSensorNames, highLevelControllerParameters,
                                                                                                    walkingControllerParameters, capturePointPlannerParameters);
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultStandPrepControlState();
      controllerFactory.useDefaultStandReadyControlState();
      controllerFactory.useDefaultStandTransitionControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.useDefaultFreezeControlState();

      controllerFactory.addRequestableTransition(STAND_PREP_STATE, FREEZE_STATE);
      controllerFactory.addFinishedTransition(STAND_PREP_STATE, STAND_READY);

      controllerFactory.addRequestableTransition(STAND_READY, FREEZE_STATE);
      controllerFactory.addRequestableTransition(STAND_READY, STAND_TRANSITION_STATE);

      controllerFactory.addRequestableTransition(STAND_TRANSITION_STATE, FREEZE_STATE);
      controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING_STATE);

      controllerFactory.addRequestableTransition(WALKING_STATE, FREEZE_STATE);

      controllerFactory.setInitialState(STAND_PREP_STATE);
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
      initialize();
      avatarSimulation.start();
   }

   /**
    * used to inject anything you need into scs before the sim starts
    */
   public void initialize()
   {

   }

   /**
    * Creates the momentum controller factory.
    */
   public HighLevelHumanoidControllerFactory getControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                                                      SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                                                      WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters,
                                                                      HighLevelControllerParameters highLevelControllerParameters)
   {
      return new HighLevelHumanoidControllerFactory(contactableBodiesFactory, footForceSensorNames, footContactSensorNames, wristSensorNames, highLevelControllerParameters,
                                                    walkingControllerParameters, capturePointPlannerParameters);
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      if (avatarSimulation.getMomentumBasedControllerFactory() != null)
         avatarSimulation.getMomentumBasedControllerFactory().attachControllerFailureListener(listener);
      else
         avatarSimulation.getHighLevelHumanoidControllerFactory().attachControllerFailureListener(listener);
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
   }
}
