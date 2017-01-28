package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.io.IOException;
import java.io.InputStream;
import java.util.Random;

public class StepAdjustmentDemoHelper
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   // Increase to 10 when you want the sims to run a little faster and don't need the data.
   private final int recordFrequencySpeedup = 1;

   private final SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private DRCSimulationStarter simulationStarter;

   private final PushRobotController pushRobotController;

   private final String scriptName;

   private final AtlasRobotModel robotModel;
   private double totalMass;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private double swingTime;
   private double transferTime;
   private double initialTransferTime;

   public StepAdjustmentDemoHelper(AtlasRobotModel robotModel, String scriptName)
   {
      this.scriptName = scriptName;
      this.robotModel = robotModel;

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      PacketCommunicator controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());

      try
      {
         controllerCommunicator.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      simulationStarter = new DRCSimulationStarter(robotModel, flatGround);
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      if (selectedLocation != null)
         simulationStarter.setStartingLocation(selectedLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableNetworkProcessor(false);

      simulationStarter.createSimulation(networkProcessorParameters, true, false);

      scs = simulationStarter.getSimulationConstructionSet();
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      simulationStarter.attachControllerFailureListener(blockingSimulationRunner.createControllerFailureListener());

      pushRobotController = new PushRobotController(simulationStarter.getSDFRobot(), fullRobotModel);
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      setupTest();
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public double getSwingTime()
   {
      return swingTime;
   }

   public double getTransferTime()
   {
      return transferTime;
   }

   public double getInitialTransferTime()
   {
      return initialTransferTime;
   }

   public StateTransitionCondition getSingleSupportStartCondition(RobotSide robotSide)
   {
      return singleSupportStartConditions.get(robotSide);
   }

   public StateTransitionCondition getDoubleSupportStartCondition(RobotSide robotSide)
   {
      return doubleSupportStartConditions.get(robotSide);
   }

   public void simulateAndBlock(double simulationTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      blockingSimulationRunner.simulateAndBlock(simulationTime);
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime)
   {
      try
      {
         simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         PrintTools.error(this, e.getMessage());
         return false;
      }
   }

   public void applyForceDelayed(StateTransitionCondition pushCondition, double delay, Vector3d forceDirection, double magnitude, double duration)
   {
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   }

   private void setupTest()
   {
      this.setupTest(ReferenceFrame.getWorldFrame());
   }

   private void setupTest(ReferenceFrame yawReferenceFrame)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();

      start();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final EnumYoVariable<FootControlModule.ConstraintType> footConstraintType = (EnumYoVariable<FootControlModule.ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
               footPrefix + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingStateEnum> walkingState = (EnumYoVariable<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController",
               "walkingState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      setupCamera(scs);
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      initialTransferTime = robotModel.getCapturePointPlannerParameters().getDoubleSupportInitialTransferDuration();
      ThreadTools.sleep(1000);
   }

   private void start()
   {
      if (scriptName != null && !scriptName.isEmpty())
      {
         simulateAndBlockAndCatchExceptions(0.001);
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
      }
   }


   private void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      ScriptBasedControllerCommandGenerator scriptBasedControllerCommandGenerator = simulationStarter.getScriptBasedControllerCommandGenerator();
      scriptBasedControllerCommandGenerator.loadScriptFile(scriptInputStream, referenceFrame);
   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(0.0, 0.0, 0.89);
      Point3d cameraPosition = new Point3d(10.0, 2.0, 1.37);

      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");

      Random randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent = new Random();
      Vector3d randomCameraOffset = RandomTools.generateRandomVector(randomForSlightlyMovingCameraSoThatYouTubeVideosAreDifferent, 0.05);
      cameraFix.add(randomCameraOffset);

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(false, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<FootControlModule.ConstraintType> footConstraintType;

      public SingleSupportStartCondition(EnumYoVariable<FootControlModule.ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean checkCondition()
      {
         return footConstraintType.getEnumValue() == FootControlModule.ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(EnumYoVariable<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean checkCondition()
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }
}
