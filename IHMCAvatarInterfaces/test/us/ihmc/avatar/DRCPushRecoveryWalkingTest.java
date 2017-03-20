package us.ihmc.avatar;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCPushRecoveryWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private double swingTime = 0.6;
   private double transferTime = 0.25;

   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private PushRobotController pushRobotController;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 750.0;
      double duration = 0.04;
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.RIGHT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      // push the robot again with new parameters
      forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      magnitude = 700.0;
      duration = 0.05;
      side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(-0.5, 1.0, 0.0);
      double magnitude = 800;
      double duration = 0.05;
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.5, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.05;
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);

      // push the robot again with new parameters
      forceDirection = new Vector3D(0.5, -1.0, 0.0);
      magnitude = 700.0;
      duration = 0.05;
      double percentInSwing = 0.4;
      side = RobotSide.RIGHT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.05;
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);

      // push the robot again with new parameters
      forceDirection = new Vector3D(0.0, 1.0, 0.0);
      magnitude = 600.0;
      duration = 0.05;
      double percentInSwing = 0.4;
      side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
   }

   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 700.0;
      double duration = 0.05;

      // This doesn't work for 0.5 or higher. We need to do more development to get this working better. So for now, just stick with 0.4.
      // 0.9 * Math.random();
      double percentInTransferState = 0.4;
      PrintTools.info(this, "percentInTransferState = " + percentInTransferState);
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, getRobotModel());
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.6, 0.0, 0.6), new Point3D(10.0, 3.0, 3.0));

      // get YoVariables
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
         swingStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         swingFinishConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      enable.set(true);
   }

   private void walkForward() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 10;

      ReferenceFrame pelvisFrame = drcSimulationTestHelper.getSDFFullRobotModel().getPelvis().getBodyFixedFrame();

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         FramePoint location = new FramePoint(pelvisFrame, footstepX, footstepY, 0.0);
         location.changeFrame(ReferenceFrame.getWorldFrame());
         location.setZ(0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, location.getPoint(), orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footsteps.add(footstepData);
      }

      drcSimulationTestHelper.send(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   private void testPush(Vector3D forceDirection, double magnitude, double duration, double percentInState, RobotSide side,
         SideDependentList<StateTransitionCondition> condition, double stateTime)
               throws SimulationExceededMaximumTimeException
   {
      walkForward();
      double delay = stateTime * percentInState;
      pushRobotController.applyForceDelayed(condition.get(side), delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0));
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
}
