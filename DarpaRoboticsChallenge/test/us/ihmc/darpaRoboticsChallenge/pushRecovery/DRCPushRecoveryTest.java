package us.ihmc.darpaRoboticsChallenge.pushRecovery;

import static org.junit.Assert.*;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingState;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public abstract class DRCPushRecoveryTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static String script = "scripts/ExerciseAndJUnitScripts/walkingPushTestScript.xml";
   private static double simulationTime = 6.0;

   private DRCPushRobotController pushRobotController;

   private double swingTime, transferTime;
   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage();
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage();
   }

   protected abstract DRCRobotModel getRobotModel();

   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 172343)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.1;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 172343)
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, false, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.25 * swingTime;

      // push parameters:
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 500.0;
      double duration = 0.1;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 164603)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      setupTest(script, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * transferTime;

      // push parameters:
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 500.0;
      double duration = 0.1;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 163619)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      double magnitude = 400.0;
      double duration = 0.15;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 25.0)
   @Test(timeout = 163619)
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      double magnitude = 400.0;
      double duration = 0.15;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 35.0)
   @Test(timeout = 163619)
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0;
      double duration = 2.0;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 25.0)
   @Test(timeout = 163619)
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(-1.0, 0.0, 0.0);
      double magnitude = 100.0;
      double duration = 2.0;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      try
      {
         drcSimulationTestHelper.simulateAndBlock(duration + 2.0);
         fail("Robot fall has not been detected");
      }
      catch (ControllerFailureException e)
      {
         // Good
      }
   }

   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 163619)
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(-1.0, 0.0, 0.0);
      double magnitude = 100.0;
      double duration = 2.0;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 25.0)
   @Test(timeout = 163619)
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0;
      double duration = 2.0;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 35.0)
   @Test(timeout = 163619)
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(-1.0, 0.0, 0.0);
      double magnitude = 100.0;
      double duration = 2.0;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0);
      assertTrue(success);
   }

   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 163619)
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      RobotSide footSide = RobotSide.LEFT;
      FramePose footPose = new FramePose(drcSimulationTestHelper.getDRCSimulationFactory().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.translate(0.0, 0.0, 0.2);
      Point3d desiredFootPosition = new Point3d();
      Quat4d desiredFootOrientation = new Quat4d();
      footPose.getPose(desiredFootPosition, desiredFootOrientation);
      FootPosePacket footPosePacket = new FootPosePacket(footSide, desiredFootPosition, desiredFootOrientation, 0.6);
      drcSimulationTestHelper.send(footPosePacket);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 200.0;
      double duration = 0.2;

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);
   }

   private void setupTest(String scriptName, boolean enablePushRecoveryControlModule, boolean enablePushRecoveryOnFailure)
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", scriptName, selectedLocation,
            simulationTestingParameters, getRobotModel());
      SDFFullRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      pushRobotController = new DRCPushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      // get rid of this once push recovery is enabled by default
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      enable.set(enablePushRecoveryControlModule);
      BooleanYoVariable enableOnFailure = (BooleanYoVariable) scs.getVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
            "enablePushRecoveryOnFailure");
      enableOnFailure.set(enablePushRecoveryOnFailure);

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = fullRobotModel.getFoot(robotSide).getName();
         @SuppressWarnings("unchecked")
         final EnumYoVariable<ConstraintType> footConstraintType = (EnumYoVariable<ConstraintType>) scs.getVariable(prefix + "FootControlModule", prefix
               + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingState> walkingState = (EnumYoVariable<WalkingState>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");

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
      Point3d cameraFix = new Point3d(0.0, 0.0, 0.89);
      Point3d cameraPosition = new Point3d(10.0, 2.0, 1.37);

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
      private final EnumYoVariable<WalkingState> walkingState;
      private final RobotSide side;

      public DoubleSupportStartCondition(EnumYoVariable<WalkingState> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean checkCondition()
      {
         if (side == RobotSide.LEFT)
         {
            return walkingState.getEnumValue() == WalkingState.DOUBLE_SUPPORT || walkingState.getEnumValue() == WalkingState.TRANSFER_TO_LEFT_SUPPORT;
         }
         else
         {
            return walkingState.getEnumValue() == WalkingState.DOUBLE_SUPPORT || walkingState.getEnumValue() == WalkingState.TRANSFER_TO_RIGHT_SUPPORT;
         }
      }
   }
}
