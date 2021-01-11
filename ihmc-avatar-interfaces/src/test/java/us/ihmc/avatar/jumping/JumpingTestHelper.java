package us.ihmc.avatar.jumping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.jumpingSimulation.JumpingSimulationController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingGoal;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.LocalObjectCommunicator;
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
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class JumpingTestHelper
{
   private final CommandInputManager commandInputManager;
   private final SimulationConstructionSet scs;

   private final BlockingSimulationRunner blockingSimulationRunner;

   private final YoBoolean shouldBeSquatting;
   private final SimulationTestingParameters simulationTestingParameters;

   private static final double gravityZ = 9.81;

   public JumpingTestHelper(SimulationTestingParameters simulationTestingParameters,
                            DRCRobotModel robotModel,
                            DRCRobotInitialSetup initialSetup,
                            String parameterResourceName)
   {
      this.simulationTestingParameters = simulationTestingParameters;
      List<Class<? extends Command<?, ?>>> availableCommands = new ArrayList<>();
      availableCommands.add(JumpingGoal.class);
      commandInputManager = new CommandInputManager(availableCommands);

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

      scs = new SimulationConstructionSet(humanoidRobot, simulationTestingParameters);
      scsInitialSetup.initializeSimulation(scs);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setDT(robotModel.getSimulateDT(), 1);

      shouldBeSquatting = ((YoBoolean) scs.findVariable("ShouldBeSquatting"));

      if (scs.getGUI() != null)
      {
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.setShowOnStart(true);
         plotterFactory.setVariableNameToTrack("centerOfMass");
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();
      }

      PushRobotController pushRobotController = new PushRobotController(humanoidRobot, fullRobotModel);
      scs.addYoGraphic(pushRobotController.getForceVisualizer());
      pushRobotController.setPushDuration(0.05);
      pushRobotController.setPushForceDirection(new Vector3D(1.0, 0.0, 0.0));
      pushRobotController.setPushForceMagnitude(5000.0);
      pushRobotController.addPushButtonToSCS(scs);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      simulationController.attachControllerFailureListener(direction -> blockingSimulationRunner.notifyControllerHasFailed());
      simulationController.attachControllerFailureListener(direction -> notifyControllerHasFailed());
   }

   public void destroySimulation()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
      }
   }

   private final AtomicBoolean hasControllerFailed = new AtomicBoolean(false);

   public void notifyControllerHasFailed()
   {
      hasControllerFailed.set(true);
      scs.stop();
   }

   public SimulationConstructionSet getSCS()
   {
      return scs;
   }

   public void startSimulation()
   {
      scs.startOnAThread();;
   }

   public BlockingSimulationRunner getBlockingSimulationRunner()
   {
      return blockingSimulationRunner;
   }

   public <C extends Command<C, ?>> void submitCommand(C command)
   {
      commandInputManager.submitCommand(command);
   }

   public void triggerSquat(boolean squat)
   {
      shouldBeSquatting.set(squat);
   }
}
