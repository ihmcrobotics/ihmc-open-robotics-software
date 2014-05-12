package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class DRCFlatGroundWalkingTrack
{   
   private static final boolean START_YOVARIABLE_SERVER = false; 
   private final DRCSimulationFactory drcSimulation;
   private final YoVariableServer robotVisualizer;

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, boolean useVelocityAndHeadingScript, AutomaticSimulationRunner automaticSimulationRunner,
                                    int simulationDataBufferSize, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      WalkingControllerParameters walkingControlParameters = model.getWalkingControlParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

//    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      boolean useFastTouchdowns = false;

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControlParameters);
      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory =
         new FlatGroundWalkingHighLevelHumanoidControllerFactory(footstepTimingParameters, walkingControlParameters, armControllerParameters, useVelocityAndHeadingScript, useFastTouchdowns);

      if (cheatWithGroundHeightAtForFootstep)
      {
         highLevelHumanoidControllerFactory.setupForCheatingUsingGroundHeightAtForFootstepProvider(scsInitialSetup.getGroundProfile());
      }

      if (START_YOVARIABLE_SERVER)
      {
         throw new RuntimeException("Something about chickens and eggs");
         //robotVisualizer = new YoVariableServer(robotInterface.getRobot().getRobotsYoVariableRegistry(), DRCLocalConfigParameters.DEFAULT_YOVARIABLE_SERVER_PORT, DRCConfigParameters.ESTIMATOR_DT, dynamicGraphicObjectsListRegistry);
      }
      else
      {
         robotVisualizer = null;
      }
      
      SideDependentList<String> footForceSensorNames = model.getSensorInformation().getFeetForceSensorNames();
      
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, DRCConfigParameters.contactTresholdForceForSCS, footForceSensorNames);
      drcSimulation = new DRCSimulationFactory(model, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup, null, robotVisualizer);

      // add other registries
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);

      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }
      
      if (robotVisualizer != null)
      {
         robotVisualizer.start();
      }
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCSimulationFactory getDrcSimulation()
   {
      return drcSimulation;
   }

   public YoVariableServer getRobotVisualizer()
   {
      return robotVisualizer;
   }
}
