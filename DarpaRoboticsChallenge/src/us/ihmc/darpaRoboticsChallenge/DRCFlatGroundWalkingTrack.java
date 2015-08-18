package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.YoVariableVariousWalkingProviderFactory;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;


import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoUtilities.time.GlobalTimer;

public class DRCFlatGroundWalkingTrack
{
   // looking for CREATE_YOVARIABLE_WALKING_PROVIDERS ?  use the second constructor and pass in WalkingProvider = YOVARIABLE_PROVIDER

   private final DRCSimulationFactory drcSimulation;
   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model, 
            WalkingProvider.VELOCITY_HEADING_COMPONENT); // should always be committed as VELOCITY_HEADING_COMPONENT
   }

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model, WalkingProvider walkingProvider)
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
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters,
            HighLevelState.WALKING);

      HeightMap heightMapForCheating = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForCheating = scsInitialSetup.getHeightMap();
      }

      VariousWalkingProviderFactory variousWalkingProviderFactory=null;
      switch(walkingProvider)
      {
      case YOVARIABLE:
         variousWalkingProviderFactory = new YoVariableVariousWalkingProviderFactory();
         break;
      case VELOCITY_HEADING_COMPONENT:
         variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(useVelocityAndHeadingScript, heightMapForCheating, model.getControllerDT());
         break;
      case  DATA_PRODUCER:
         throw new RuntimeException("Please run ObstacleCourseDemo instead");
      }

      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      
      drcSimulation = new DRCSimulationFactory(model, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      drcSimulation.start();
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      drcSimulation.getControllerFactory().attachControllerFailureListener(listener);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCSimulationFactory getDrcSimulation()
   {
      return drcSimulation;
   }

   public void destroySimulation()
   {
      if (drcSimulation != null)
      {
         drcSimulation.dispose();
      }
      GlobalTimer.clearTimers();
   }
}
