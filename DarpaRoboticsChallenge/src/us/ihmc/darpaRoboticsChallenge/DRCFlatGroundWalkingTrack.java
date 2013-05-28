package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCSimDRCRobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCFlatGroundWalkingTrack
{
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;

   public DRCFlatGroundWalkingTrack(DRCRobotWalkingControllerParameters drcControlParameters, DRCRobotInterface robotInterface,
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
         dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      else
         dynamicGraphicObjectsListRegistry = null;
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      boolean useFastTouchdowns = DRCConfigParameters.USE_GAZEBO_PHYSICS;

      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory =
         new FlatGroundWalkingHighLevelHumanoidControllerFactory(drcControlParameters, useVelocityAndHeadingScript, useFastTouchdowns);

      if (cheatWithGroundHeightAtForFootstep)
      {
         highLevelHumanoidControllerFactory.setupForCheatingUsingGroundHeightAtForFootstepProvider(scsInitialSetup.getGroundProfile());
      }

      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, DRCConfigParameters.USE_GAZEBO_PHYSICS);
      drcSimulation = DRCSimulationFactory.createSimulation(controllerFactory, null, robotInterface, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      // add other registries
      if (dynamicGraphicObjectsListRegistry != null)
         drcSimulation.addAdditionalDynamicGraphicObjectsListRegistries(dynamicGraphicObjectsListRegistry);
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);

      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public static void main(String[] args) throws JSAPException
   {
      AutomaticSimulationRunner automaticSimulationRunner = null;

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

//    DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT_Z_NEGATIVE_TWO);
//    RobotInitialSetup<SDFRobot> robotInitialSetup = new SquaredUpDRCRobotInitialSetup(-2.0);
      DRCRobotInterface robotInterface = new PlainDRCRobot(DRCRobotModel.getDefaultRobotModel(), false);
      final double groundHeight = 0.0;
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(new FlatGroundProfile(groundHeight), robotInterface.getSimulateDT());
      RobotInitialSetup<SDFRobot> robotInitialSetup = new DRCSimDRCRobotInitialSetup(groundHeight);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCRobotWalkingControllerParameters drcControlParameters = new DRCRobotWalkingControllerParameters();

      new DRCFlatGroundWalkingTrack(drcControlParameters, robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript,
                                    automaticSimulationRunner, 0.005, 16000, cheatWithGroundHeightAtForFootstep);
   }


}
