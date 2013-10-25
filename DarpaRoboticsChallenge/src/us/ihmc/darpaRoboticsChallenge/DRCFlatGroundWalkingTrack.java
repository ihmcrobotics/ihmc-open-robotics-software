package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.visualization.SliderBoardFactory;
import us.ihmc.atlas.visualization.WalkControllerSliderBoard;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.SimulatedModelCorruptorDRCRobotInterface;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCSimDRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.remote.RemoteAtlasVisualizer;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.Pair;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCFlatGroundWalkingTrack
{
   private static final boolean START_YOVARIABLE_SERVER = true; 
   
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;

   public DRCFlatGroundWalkingTrack(DRCRobotWalkingControllerParameters drcControlParameters, ArmControllerParameters armControllerParameters, 
         DRCRobotInterface robotInterface,
                                    RobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
                                    boolean useVelocityAndHeadingScript, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
                                    int simulationDataBufferSize, boolean cheatWithGroundHeightAtForFootstep)
   {
//    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
      if (guiInitialSetup.isGuiShown())
         dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry(false);
      else
         dynamicGraphicObjectsListRegistry = null;
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      boolean useFastTouchdowns = false;

      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory =
         new FlatGroundWalkingHighLevelHumanoidControllerFactory(drcControlParameters, armControllerParameters, useVelocityAndHeadingScript, useFastTouchdowns);

      if (cheatWithGroundHeightAtForFootstep)
      {
         highLevelHumanoidControllerFactory.setupForCheatingUsingGroundHeightAtForFootstepProvider(scsInitialSetup.getGroundProfile());
      }

      YoVariableServer robotVisualizer = null;
      
      if (START_YOVARIABLE_SERVER)
      {
         robotVisualizer = new YoVariableServer(robotInterface.getRobot().getRobotsYoVariableRegistry(), RemoteAtlasVisualizer.defaultPort, DRCConfigParameters.ESTIMATE_DT, dynamicGraphicObjectsListRegistry);
      }
      
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, false);
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> humanoidSimulation = DRCSimulationFactory.createSimulation(controllerFactory, null, robotInterface, robotInitialSetup, scsInitialSetup, guiInitialSetup, null, robotVisualizer, dynamicGraphicObjectsListRegistry);
      drcSimulation = humanoidSimulation.first();

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

   public static void main(String[] args) throws JSAPException
   {
      AutomaticSimulationRunner automaticSimulationRunner = null;

      SliderBoardFactory sliderBoardFactory = WalkControllerSliderBoard.getFactory();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, sliderBoardFactory);

//    DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT_Z_NEGATIVE_TWO);
//    RobotInitialSetup<SDFRobot> robotInitialSetup = new SquaredUpDRCRobotInitialSetup(-2.0);
      DRCRobotInterface robotInterface = new PlainDRCRobot(DRCRobotModel.getDefaultRobotModel(), false);
      
      if (DRCConfigParameters.CORRUPT_SIMULATION_MODEL)
      {
         robotInterface = new SimulatedModelCorruptorDRCRobotInterface(robotInterface);
      }
      
      final double groundHeight = 0.0;
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(new FlatGroundProfile(groundHeight), robotInterface.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.3;
      RobotInitialSetup<SDFRobot> robotInitialSetup = new DRCSimDRCRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCRobotWalkingControllerParameters drcControlParameters = new DRCRobotWalkingControllerParameters();
      DRCRobotArmControllerParameters armControlParameters = new DRCRobotArmControllerParameters();
      
      new DRCFlatGroundWalkingTrack(drcControlParameters, armControlParameters, robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript,
                                    automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000, cheatWithGroundHeightAtForFootstep);
   }


}
