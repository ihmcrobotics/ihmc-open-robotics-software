package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCFlatGroundWalkingTrack
{
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;

   public DRCFlatGroundWalkingTrack(DRCRobotModel robotModel, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup, 
         AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
         int simulationDataBufferSize, boolean doChestOrientationControl)
   {
      RobotInitialSetup<SDFRobot> robotInitialSetup = new SquaredUpDRCRobotInitialSetup();
      WalkingControllerParameters drcRobotParameters = new DRCRobotWalkingControllerParameters();

//      scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      double inPlaceWidth = 0.25;
      double maxStepLength = 0.35;
      double minStepWidth = 0.15;
      double maxStepWidth = 0.4;
      double stepPitch = 0.0;
      boolean useVelocityAndHeadingScript = true;
      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = new FlatGroundWalkingHighLevelHumanoidControllerFactory(drcRobotParameters,
            inPlaceWidth, maxStepLength, minStepWidth, maxStepWidth, stepPitch, useVelocityAndHeadingScript);

      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.getDefaultRobotModel());
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, false);
      drcSimulation = DRCSimulationFactory.createSimulation(jointMap, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup);

      // add other registries
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

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      new DRCFlatGroundWalkingTrack(DRCRobotModel.getDefaultRobotModel(), guiInitialSetup, scsInitialSetup, automaticSimulationRunner, 0.005, 16000, true);
   }

   
}
