package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.RemoteAtlasVisualizer;
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
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCFlatGroundWalkingTrack
{   
   private static final boolean START_YOVARIABLE_SERVER = false; 
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;
   private final DRCController drcController;
   private final YoVariableServer robotVisualizer;

   public DRCFlatGroundWalkingTrack(DRCRobotInterface robotInterface, DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, boolean useVelocityAndHeadingScript, AutomaticSimulationRunner automaticSimulationRunner,
                                    double timePerRecordTick, int simulationDataBufferSize, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      WalkingControllerParameters walkingControlParameters = model.getWalkingControlParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

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

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForFastWalkingInSimulation(walkingControlParameters);
      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory =
         new FlatGroundWalkingHighLevelHumanoidControllerFactory(footstepTimingParameters, walkingControlParameters, armControllerParameters, useVelocityAndHeadingScript, useFastTouchdowns);

      if (cheatWithGroundHeightAtForFootstep)
      {
         highLevelHumanoidControllerFactory.setupForCheatingUsingGroundHeightAtForFootstepProvider(scsInitialSetup.getGroundProfile());
      }

      if (START_YOVARIABLE_SERVER)
      {
         robotVisualizer = new YoVariableServer(robotInterface.getRobot().getRobotsYoVariableRegistry(), RemoteAtlasVisualizer.defaultPort, DRCConfigParameters.ESTIMATOR_DT, dynamicGraphicObjectsListRegistry);
      }
      else
      {
         robotVisualizer = null;
      }
      
      SideDependentList<String> footForceSensorNames = new SideDependentList<>();
      for(RobotSide robotSide : RobotSide.values)
      {
         footForceSensorNames.put(robotSide, robotInterface.getJointMap().getJointBeforeFootName(robotSide));
      }
      
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, DRCConfigParameters.contactTresholdForceForSCS, footForceSensorNames);
      Pair<HumanoidRobotSimulation<SDFRobot>, DRCController> humanoidSimulation = DRCSimulationFactory.createSimulation(controllerFactory, null,
            robotInterface, robotInitialSetup, scsInitialSetup, guiInitialSetup, null, robotVisualizer, dynamicGraphicObjectsListRegistry, model);
      drcSimulation = humanoidSimulation.first();
      drcController = humanoidSimulation.second();

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

   public DRCController getDrcController()
   {
      return drcController;
   }

   public YoVariableServer getRobotVisualizer()
   {
      return robotVisualizer;
   }
}
