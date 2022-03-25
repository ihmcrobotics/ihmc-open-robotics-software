package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarFootstepQueueingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   @Test
   public void testTwoPlans() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscriptionTypeNamed(simulationTestHelper.getROS2Node(),
                                                    FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                    (p) ->
                                                    {
                                                       if (FootstepStatus.fromByte(p.takeNextData().getFootstepStatus()) == FootstepStatus.STARTED)
                                                       {
                                                          stepCounter.incrementAndGet();
                                                       }
                                                    });

      DRCRobotModel robotModel = getRobotModel();

      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      ThreadTools.sleep(1000);

      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = new FootstepDataListMessage();

      int firstNumberofSteps = 3;
      double stepX = 0.0;
      for (int currentStep = 0; currentStep < firstNumberofSteps; currentStep++)
      {
         Point3D footLocation = new Point3D(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footMessage.getFootstepDataList().add().set(createFootstepDataMessage(footLocation, footOrientation, side));
         side = side.getOppositeSide();

         stepX += stepLength;
      }

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      simulationTestHelper.simulateAndWait(1.0);
      simulationTestHelper.publishToController(footMessage);

      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps - 1);

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      footMessage = new FootstepDataListMessage();
      footMessage.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());

      int secondNumberOfSteps = 3;
      for (int currentStep = 0; currentStep < secondNumberOfSteps; currentStep++)
      {
         Point3D footLocation = new Point3D(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footMessage.getFootstepDataList().add().set(createFootstepDataMessage(footLocation, footOrientation, side));
         side = side.getOppositeSide();

         stepX += stepLength;
      }

      simulationTestHelper.publishToController(footMessage);

      simulationTime = transfer + (transfer + swing) * (secondNumberOfSteps + 1);

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
      assertEquals(firstNumberofSteps + secondNumberOfSteps, stepCounter.get());
   }

   @Test
   public void testQueuedStepsSequentialWithMessageTools() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscriptionTypeNamed(simulationTestHelper.getROS2Node(),
                                                    FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                    (p) ->
                                                    {
                                                       if (FootstepStatus.fromByte(p.takeNextData().getFootstepStatus()) == FootstepStatus.STARTED)
                                                       {
                                                          stepCounter.incrementAndGet();
                                                       }
                                                    });

      DRCRobotModel robotModel = getRobotModel();

      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      ThreadTools.sleep(1000);

      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      simulationTestHelper.simulateAndWait(1.0);

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      int firstNumberofSteps = 4;
      double stepX = 0.0;
      for (int currentStep = 0; currentStep < firstNumberofSteps; currentStep++)
      {

         ExecutionMode currentExecutionMode = ExecutionMode.QUEUE;
         if (currentStep == 0)
         {
            currentExecutionMode = ExecutionMode.OVERRIDE;
         }

         Point3D footLocation = new Point3D(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);

         //make a single foot step
         FootstepDataMessage currentFootStep = createFootstepDataMessage(footLocation, footOrientation, side);
         //add that single foot step to a footstep list
         ArrayList<FootstepDataMessage> currentFootStepList = new ArrayList<FootstepDataMessage>();
         currentFootStepList.add(currentFootStep);

         //use the HumanoidMessageTools to generate footstep data list message
         FootstepDataListMessage footStepToSend = HumanoidMessageTools.createFootstepDataListMessage(currentFootStepList,
                                                                                                     swing,
                                                                                                     transfer,
                                                                                                     currentExecutionMode);

         side = side.getOppositeSide();

         simulationTestHelper.publishToController(footStepToSend);

         stepX += stepLength;

         // send a new footstep when the current footstep is halfway done
         double simulationTime = (transfer + swing);

         assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      }

      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps - 1) - (((transfer + swing)) * firstNumberofSteps - 1);

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      assertEquals(firstNumberofSteps, stepCounter.get());
   }

   @Test
   public void testOnlyQueuedStepsWithTinySims() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscriptionTypeNamed(simulationTestHelper.getROS2Node(),
                                                    FootstepStatusMessage.class,
                                                    ROS2Tools.getControllerOutputTopic(getSimpleRobotName()),
                                                    (p) ->
                                                    {
                                                       if (FootstepStatus.fromByte(p.takeNextData().getFootstepStatus()) == FootstepStatus.STARTED)
                                                       {
                                                          stepCounter.incrementAndGet();
                                                       }
                                                    });

      DRCRobotModel robotModel = getRobotModel();

      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      ThreadTools.sleep(1000);

      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      simulationTestHelper.simulateAndWait(1.0);

      int firstNumberofSteps = 4;
      double stepX = 0.0;
      for (int currentStep = 0; currentStep < firstNumberofSteps; currentStep++)
      {
         FootstepDataListMessage footMessage = new FootstepDataListMessage();
         footMessage.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());

         Point3D footLocation = new Point3D(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footMessage.getFootstepDataList().add().set(createFootstepDataMessage(footLocation, footOrientation, side));
         side = side.getOppositeSide();

         simulationTestHelper.publishToController(footMessage);
         simulationTestHelper.simulateAndWait(0.05);

         stepX += stepLength;
      }

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps);

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      assertEquals(firstNumberofSteps, stepCounter.get());
   }

   private FootstepDataMessage createFootstepDataMessage(Point3D stepLocation, Quaternion orient, RobotSide robotSide)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(orient);
      footstepData.setRobotSide(robotSide.toByte());
      return footstepData;
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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
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
