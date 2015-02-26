package us.ihmc.darpaRoboticsChallenge.pushRecovery;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingState;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
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
   }

   protected abstract DRCRobotModel getRobotModel();

	@AverageDuration(duration = 34.5)
	@Test(timeout = 172343)
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(script);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      
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
      
      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration(duration = 32.9)
	@Test(timeout = 164603)
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(script);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      
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
      
      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration(duration = 32.7)
	@Test(timeout = 163619)
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(null);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      
      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;
      
      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      double magnitude = 500.0;
      double duration = 0.1;
      
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration); 
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   private void setupTest(String scriptName)
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", scriptName, selectedLocation, simulationTestingParameters, getRobotModel());
      SDFFullRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      pushRobotController = new DRCPushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());
      
      // get rid of this once push recovery is enabled by default
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      BooleanYoVariable enableDS = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule","enablePushRecoveryFromDoubleSupport");
      enable.set(true);
      enableDS.set(true);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = fullRobotModel.getFoot(robotSide).getName();
         final EnumYoVariable<ConstraintType> footConstraintType = (EnumYoVariable<ConstraintType>) scs.getVariable(prefix + "FootControlModule", prefix + "State");
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
