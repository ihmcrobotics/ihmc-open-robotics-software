package us.ihmc.avatar.pushRecovery;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.InputStream;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCPushRecoveryTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static String script = "scripts/ExerciseAndJUnitScripts/walkingPushTestScript.xml";

   private static double simulationTime = 6.0;

   private PushRobotController pushRobotController;

   private double swingTime, transferTime;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();

   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      singleSupportStartConditions = null;
      doubleSupportStartConditions = null;
      pushRobotController = null;
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract DRCRobotModel getRobotModel();

   @ContinuousIntegrationTest(estimatedDuration =  20.0)
   @Test(timeout = 120000)
   public void testPushICPOptimiWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 600.0 * getForceScale();
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }


   @ContinuousIntegrationTest(estimatedDuration = 23.7)
   @Test(timeout = 120000)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 550.0 * getForceScale();
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 22.3)
   @Test(timeout = 110000)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, false, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.25 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 450.0 * getForceScale();
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.1)
   @Test(timeout = 110000)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * transferTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 450.0 * getForceScale();
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.5)
   @Test(timeout = 110000)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 350.0 * getForceScale();
      double duration = 0.15;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.4)
   @Test(timeout = 110000)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 350.0 * getForceScale();
      double duration = 0.15;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 18.7)
   @Test(timeout = 93000)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0 * getForceScale();
      double duration = 1.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 13.8)
   @Test(timeout = 69000)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 160.0 * getForceScale();
      double duration = 3.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      assertFalse(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 27.2)
   @Test(timeout = 140000)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 100.0 * getForceScale();
      double duration = 1.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0 * getForceScale();
      double duration = 1.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 18.6)
   @Test(timeout = 93000)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 100.0 * getForceScale();
      double duration = 1.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 16.9)
   @Test(timeout = 84000)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose footPose = new FramePose(
            drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.translate(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.getPose(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = new FootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.send(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 180.0 * getForceScale();
      double duration = 0.2;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5));
   }

   private void setupTest(String scriptName, boolean enablePushRecoveryControlModule, boolean enablePushRecoveryOnFailure) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, getRobotModel());
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      if (scriptName != null && !scriptName.isEmpty())
      {
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001));
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
      }

      // get rid of this once push recovery is enabled by default
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      enable.set(enablePushRecoveryControlModule);
      BooleanYoVariable enableOnFailure = (BooleanYoVariable) scs.getVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
            "enablePushRecoveryOnFailure");
      enableOnFailure.set(enablePushRecoveryOnFailure);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final EnumYoVariable<ConstraintType> footConstraintType = (EnumYoVariable<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
               footPrefix + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingStateEnum> walkingState = (EnumYoVariable<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController",
               "walkingState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      setupCamera(scs);
      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      ThreadTools.sleep(1000);
   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(EnumYoVariable<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean checkCondition()
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
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

   public double getForceScale()
   {
      return 1.0;
   }
}
