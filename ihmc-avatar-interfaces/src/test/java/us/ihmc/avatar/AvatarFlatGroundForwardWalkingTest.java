package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarFlatGroundForwardWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;

   private PushRobotControllerSCS2 pushRobotController;
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

   public double getMaxPlanICPChange()
   {
      return 0.08;
   }

   @Test
   public void testForwardWalk()
   {
      setupTest();
      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = getFootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(simulationTestHelper);

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         if (simulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            Point3D footLocation = new Point3D(stepLength * currentStep, side.negateIfRightSide(stepWidth / 2), 0.0);
            rootLocations.add(new Point3D(stepLength * currentStep, 0.0, 0.0));
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }
      Point3D footLocation = new Point3D(stepLength * (numberOfSteps - 1), side.negateIfRightSide(stepWidth / 2), 0.0);
      rootLocations.add(new Point3D(stepLength * (numberOfSteps - 1), 0.0, 0.0));
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      addFootstep(footLocation, footOrientation, side, footMessage);

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      int steps = footMessage.getFootstepDataList().size();

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      simulationTestHelper.simulateAndWait(1.0);
      simulationTestHelper.publishToController(footMessage);
      double simulationTime = intitialTransfer + (transfer + swing) * steps + 1.0;

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
      controllerSpy.assertCheckpointsReached();
   }

   @Test
   public void testForwardWalkWithForceDisturbances()
   {
      setupTest();
      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = getFootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(simulationTestHelper);

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         if (simulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            Point3D footLocation = new Point3D(stepLength * currentStep, side.negateIfRightSide(stepWidth / 2), 0.0);
            rootLocations.add(new Point3D(stepLength * currentStep, 0.0, 0.0));
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }
      Point3D footLocation = new Point3D(stepLength * (numberOfSteps - 1), side.negateIfRightSide(stepWidth / 2), 0.0);
      rootLocations.add(new Point3D(stepLength * (numberOfSteps - 1), 0.0, 0.0));
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      addFootstep(footLocation, footOrientation, side, footMessage);

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      simulationTestHelper.simulateAndWait(1.0);
      simulationTestHelper.publishToController(footMessage);
      double simulationTime = 1 * footMessage.getFootstepDataList().size() + 1.0;

      boolean success;
      // Push:
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);

      success = simulationTestHelper.simulateAndWait(2.0);
      assertTrue(success);

      //      magnitude1 = 40; //TODO: overwritten
      LogTools.info("Force magnitude = " + magnitude1 + "N along " + forceDirection1.toString());
      pushRobotController.applyForceDelayed(firstPushCondition, delay1, forceDirection1, magnitude1, duration1);
      success = simulationTestHelper.simulateAndWait(2.0);
      assertTrue(success);

      //      magnitude2 = 50; //TODO:overwritten
      LogTools.info("Force magnitude = " + magnitude2 + "N along " + forceDirection2.toString());
      pushRobotController.applyForceDelayed(secondPushCondition, delay2, forceDirection2, magnitude2, duration2);
      success = simulationTestHelper.simulateAndWait(2.0);
      assertTrue(success);

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime - 6.0));
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

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      pushRobotController = null;
      robotModel = null;
      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest() 
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      LogTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.addDesiredICPContinuityAssertion(getMaxPlanICPChange());
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

      LogTools.info("Creating push controller");

      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationSession().getTime(),
                                                        simulationTestHelper.getRobot(),
                                                        fullRobotModel.getChest().getParentJoint().getName(),
                                                        new Vector3D(0, 0, z));
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(sidePrefix + "FootControlModule",
                                                                                                                      sidePrefix + "FootCurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
                                                                                                                    "walkingCurrentState");
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
