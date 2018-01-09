package us.ihmc.commonWalkingControlModules;

import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

import static org.junit.Assert.assertTrue;

public abstract class AvatarICPOptimizationPushRecoveryTestSetup
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected static double simulationTime = 10.0;

   protected PushRobotController pushRobotController;

   protected double swingTime, transferTime;
   protected double totalMass;

   protected SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   protected SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract DRCRobotModel getRobotModel();

   protected abstract double getSizeScale();

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }


   protected void setupAndRunTest(FootstepDataListMessage message) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();

      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(),
                                                    new Vector3D(0, 0, z));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      drcSimulationTestHelper.send(message);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule", footPrefix + "State");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      setupCamera(scs);
      ThreadTools.sleep(1000);
   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   protected FootstepDataListMessage createForwardWalkingFootstepMessage()
   {
      double scale = getSizeScale();

      FramePoint3D step1Location = new FramePoint3D(worldFrame, 0.5, -0.125, 0.0);
      FramePoint3D step2Location = new FramePoint3D(worldFrame, 1.0,  0.125, 0.0);
      FramePoint3D step3Location = new FramePoint3D(worldFrame, 1.5, -0.125, 0.0);
      FramePoint3D step4Location = new FramePoint3D(worldFrame, 2.0,  0.125, 0.0);
      FramePoint3D step5Location = new FramePoint3D(worldFrame, 2.5, -0.125, 0.0);
      FramePoint3D step6Location = new FramePoint3D(worldFrame, 3.0,  0.125, 0.0);

      step1Location.scale(scale);
      step2Location.scale(scale);
      step3Location.scale(scale);
      step4Location.scale(scale);
      step5Location.scale(scale);
      step6Location.scale(scale);

      FootstepDataMessage message1 = createFootstepDataMessage(RobotSide.RIGHT, step1Location);
      FootstepDataMessage message2 = createFootstepDataMessage(RobotSide.LEFT, step2Location);
      FootstepDataMessage message3 = createFootstepDataMessage(RobotSide.RIGHT, step3Location);
      FootstepDataMessage message4 = createFootstepDataMessage(RobotSide.LEFT, step4Location);
      FootstepDataMessage message5 = createFootstepDataMessage(RobotSide.RIGHT, step5Location);
      FootstepDataMessage message6 = createFootstepDataMessage(RobotSide.LEFT, step6Location);

      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      message.add(message1);
      message.add(message2);
      message.add(message3);
      message.add(message4);
      message.add(message5);
      message.add(message6);

      return message;
   }

   protected FootstepDataListMessage createSlowForwardWalkingFootstepMessage()
   {
      double scale = getSizeScale();

      FramePoint3D step1Location = new FramePoint3D(worldFrame, 0.3, -0.125, 0.0);
      FramePoint3D step2Location = new FramePoint3D(worldFrame, 0.6,  0.125, 0.0);
      FramePoint3D step3Location = new FramePoint3D(worldFrame, 0.9, -0.125, 0.0);
      FramePoint3D step4Location = new FramePoint3D(worldFrame, 1.2,  0.125, 0.0);
      FramePoint3D step5Location = new FramePoint3D(worldFrame, 1.5, -0.125, 0.0);
      FramePoint3D step6Location = new FramePoint3D(worldFrame, 1.8,  0.125, 0.0);

      step1Location.scale(scale);
      step2Location.scale(scale);
      step3Location.scale(scale);
      step4Location.scale(scale);
      step5Location.scale(scale);
      step6Location.scale(scale);

      FootstepDataMessage message1 = createFootstepDataMessage(RobotSide.RIGHT, step1Location);
      FootstepDataMessage message2 = createFootstepDataMessage(RobotSide.LEFT, step2Location);
      FootstepDataMessage message3 = createFootstepDataMessage(RobotSide.RIGHT, step3Location);
      FootstepDataMessage message4 = createFootstepDataMessage(RobotSide.LEFT, step4Location);
      FootstepDataMessage message5 = createFootstepDataMessage(RobotSide.RIGHT, step5Location);
      FootstepDataMessage message6 = createFootstepDataMessage(RobotSide.LEFT, step6Location);

      swingTime = 1.2;
      transferTime = 0.8;
      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      message.add(message1);
      message.add(message2);
      message.add(message3);
      message.add(message4);
      message.add(message5);
      message.add(message6);

      return message;
   }

   protected FootstepDataListMessage createYawingForwardWalkingFootstepMessage()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendYawRotation(0.5);
      ReferenceFrame referenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("yawing", ReferenceFrame.getWorldFrame(), transform);

      double scale = getSizeScale();

      FramePoint3D step1Location = new FramePoint3D(referenceFrame, 0.5, -0.125, 0.0);
      FramePoint3D step2Location = new FramePoint3D(referenceFrame, 1.0,  0.125, 0.0);
      FramePoint3D step3Location = new FramePoint3D(referenceFrame, 1.5, -0.125, 0.0);
      FramePoint3D step4Location = new FramePoint3D(referenceFrame, 2.0,  0.125, 0.0);
      FramePoint3D step5Location = new FramePoint3D(referenceFrame, 2.5, -0.125, 0.0);
      FramePoint3D step6Location = new FramePoint3D(referenceFrame, 3.0,  0.125, 0.0);

      FrameQuaternion orientation = new FrameQuaternion(referenceFrame);

      step1Location.scale(scale);
      step2Location.scale(scale);
      step3Location.scale(scale);
      step4Location.scale(scale);
      step5Location.scale(scale);
      step6Location.scale(scale);

      FootstepDataMessage message1 = createFootstepDataMessage(RobotSide.RIGHT, step1Location, orientation);
      FootstepDataMessage message2 = createFootstepDataMessage(RobotSide.LEFT, step2Location, orientation);
      FootstepDataMessage message3 = createFootstepDataMessage(RobotSide.RIGHT, step3Location, orientation);
      FootstepDataMessage message4 = createFootstepDataMessage(RobotSide.LEFT, step4Location, orientation);
      FootstepDataMessage message5 = createFootstepDataMessage(RobotSide.RIGHT, step5Location, orientation);
      FootstepDataMessage message6 = createFootstepDataMessage(RobotSide.LEFT, step6Location, orientation);

      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);
      message.add(message1);
      message.add(message2);
      message.add(message3);
      message.add(message4);
      message.add(message5);
      message.add(message6);

      return message;
   }

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep)
   {
      return createFootstepDataMessage(robotSide, placeToStep, new FrameQuaternion());
   }

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep, FrameQuaternion orientation)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);
      orientation.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld);
      footstepData.setOrientation(orientation.getQuaternion());
      footstepData.setRobotSide(robotSide);

      return footstepData;
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
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
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
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
