package us.ihmc.avatar.jumpingSimulation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import java.util.ArrayList;
import java.util.List;

public class JumpingSimulationFactory
{
   private final DRCRobotModel robotModel;
   private final DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup;
   private final CommandInputManager commandInputManager;

   private static final double gravityZ = 9.81;

   public JumpingSimulationFactory(DRCRobotModel robotModel, DRCRobotInitialSetup initialSetup)
   {
      this.robotModel = robotModel;
      this.initialSetup = initialSetup;

      List<Class<? extends Command<?, ?>>> availableCommands = new ArrayList<>();
      availableCommands.add(JumpingGoal.class);
      commandInputManager = new CommandInputManager(availableCommands);
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

      int controlTicksPerSimulate = ((int) (robotModel.getControllerDT() / robotModel.getSimulateDT()));
      humanoidRobot.setController(simulationController, controlTicksPerSimulate);


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

      PushRobotController pushRobotController = new PushRobotController(humanoidRobot, fullRobotModel);
      scs.addYoGraphic(pushRobotController.getForceVisualizer());
      pushRobotController.setPushDuration(0.05);
      pushRobotController.setPushForceDirection(new Vector3D(1.0, 0.0, 0.0));
      pushRobotController.setPushForceMagnitude(600.0);
      pushRobotController.addPushButtonToSCS(scs);

      return scs;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }
}
