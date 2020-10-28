package us.ihmc.avatar.jumpingSimulation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class JumpingSimulationFactory
{
   private final DRCRobotModel robotModel;
   private final DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup;
   private final CommandInputManager commandInputManager;

   private static final double gravityZ = 9.81;

   public JumpingSimulationFactory(DRCRobotModel robotModel, DRCRobotInitialSetup initialSetup, CommandInputManager commandInputManager)
   {
      this.robotModel = robotModel;
      this.initialSetup = initialSetup;
      this.commandInputManager = commandInputManager;
   }

   public SimulationConstructionSet createSimulation(String parameterResourceName)
   {
      HumanoidFloatingRootJointRobot humanoidRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      JumpingSimulationController simulationController = new JumpingSimulationController(commandInputManager,
                                                                                         robotModel,
                                                                                         fullRobotModel,
                                                                                         humanoidRobot,
                                                                                         graphicsListRegistry,
                                                                                         gravityZ,
                                                                                         getClass().getResourceAsStream(parameterResourceName));
      humanoidRobot.setController(simulationController);


      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setRunMultiThreaded(false);

      scsInitialSetup.initializeRobot(humanoidRobot, robotModel, null);
      initialSetup.initializeRobot(humanoidRobot, robotModel.getJointMap());
      initialSetup.initializeFullRobotModel(fullRobotModel);


      SimulationConstructionSet scs = new SimulationConstructionSet(humanoidRobot);
      scsInitialSetup.initializeSimulation(scs);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setDT(robotModel.getSimulateDT(), 1);

      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      return scs;
   }
}
