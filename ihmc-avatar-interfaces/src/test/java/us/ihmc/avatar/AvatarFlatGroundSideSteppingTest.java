package us.ihmc.avatar;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarFlatGroundSideSteppingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   Random random = new Random();

   private PushRobotController pushRobotController;
   private static final double GRAVITY = 9.81;
   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private int numberOfSteps;
   private double stepLength;
   private double stepWidth;
   private double totalMass;

   private double delay1;
   private double duration1;
   private double percentWeight1;
   private double magnitude1;
   private Vector3D forceDirection1;

   private double delay2;
   private double duration2;
   private double percentWeight2;
   private double magnitude2;
   private Vector3D forceDirection2;

   protected int getNumberOfSteps()
   {
      return 10;
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   protected double getTotalMass()
   {
      return fullRobotModel.getTotalMass();
   }

   protected double getForceDelay1()
   {
      return 0.5 * robotModel.getWalkingControllerParameters().getDefaultSwingTime();
   }

   protected double getForcePercentageOfWeight1()
   {
      return 0.13;
   }

   protected double getForceDuration1()
   {
      return 0.1;
   }

   protected Vector3D getForceDirection1()
   {
      return new Vector3D(0.0, -1.0, 0.0);
   }

   protected double getForceDelay2()
   {
      return 2.5 * robotModel.getWalkingControllerParameters().getDefaultSwingTime();
   }

   protected double getForcePercentageOfWeight2()
   {
      return 0.13;
   }

   protected double getForceDuration2()
   {
      return 0.1;
   }

   protected Vector3D getForceDirection2()
   {
      return new Vector3D(1.0, 0.0, 0.0);
   }

   protected FootstepDataListMessage getFootstepDataListMessage()
   {
      return new FootstepDataListMessage();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testSideStepping() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraBackView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = getFootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(drcSimulationTestHelper);

      double centerStepY = 0.0;
      for (int currentStep = 1; currentStep < numberOfSteps + 1; currentStep++)
      {
         if (drcSimulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            if(currentStep % 2 != 0)
            {
               centerStepY += stepLength;
               rootLocations.add(new Point3D(0.0, centerStepY, 0.0));
            }
            Point3D footLocation = new Point3D(0, centerStepY + side.negateIfRightSide(stepWidth / 2), 0.0);
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.send(footMessage);
      double simulationTime = 1 * footMessage.getFootstepDataList().size() + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      controllerSpy.assertCheckpointsReached();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testSideSteppingWithForceDisturbances() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraBackView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = getFootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(drcSimulationTestHelper);

      double centerStepY = 0.0;
      for (int currentStep = 1; currentStep < numberOfSteps + 1; currentStep++)
      {
         if (drcSimulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            if(currentStep % 2 != 0)
            {
               centerStepY += stepLength;
               rootLocations.add(new Point3D(0.0, centerStepY, 0.0));
            }
            Point3D footLocation = new Point3D(0, centerStepY + side.negateIfRightSide(stepWidth / 2), 0.0);
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.send(footMessage);
      double simulationTime = 1 * footMessage.getFootstepDataList().size() + 1.0;

      boolean success;
      // Push:
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      magnitude1 = 80; //TODO: overwritten
      PrintTools.info("Force magnitude = " + magnitude1 + "N along " + forceDirection1.toString());
      pushRobotController.applyForceDelayed(firstPushCondition, delay1, forceDirection1, magnitude1, duration1);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      magnitude2 = 80; //TODO:overwritten
      PrintTools.info("Force magnitude = " + magnitude2 + "N along " + forceDirection2.toString());
      pushRobotController.applyForceDelayed(secondPushCondition, delay2, forceDirection2, magnitude2, duration2);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime - 6.0));
      controllerSpy.assertCheckpointsReached();
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(orient);
      footstepData.setRobotSide(robotSide.toByte());
      message.getFootstepDataList().add().set(footstepData);
   }

   private void setupCameraBackView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return location;
         }
      };
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();
      fullRobotModel = robotModel.createFullRobotModel();

      numberOfSteps = getNumberOfSteps();
      stepLength = getStepLength();
      stepWidth = getStepWidth();
      totalMass = getTotalMass();

      delay1 = getForceDelay1();
      duration1 = getForceDuration1();
      percentWeight1 = getForcePercentageOfWeight1();
      magnitude1 = percentWeight1 * totalMass * GRAVITY;
      forceDirection1 = getForceDirection1();

      delay2 = getForceDelay2();
      duration2 = getForceDuration2();
      percentWeight2 = getForcePercentageOfWeight2();
      magnitude2 = percentWeight2 * totalMass * GRAVITY;
      forceDirection2 = getForceDirection2();

      double z = getForcePointOffsetZInChestFrame();

      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
                                                                                                    sidePrefix + "FootCurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable(WalkingHighLevelHumanoidController.class.getSimpleName(), "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      ThreadTools.sleep(1000);
   }

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean testCondition(double time)
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
