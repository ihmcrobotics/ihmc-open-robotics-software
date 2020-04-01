package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarFootstepQueueingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

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
      String className = getClass().getSimpleName();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscription(drcSimulationTestHelper.getRos2Node(), FootstepStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(getSimpleRobotName()), (p) -> {
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

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.publishToController(footMessage);

      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps - 1);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

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

      drcSimulationTestHelper.publishToController(footMessage);

      simulationTime = transfer + (transfer + swing) * (secondNumberOfSteps + 1);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      assertEquals(firstNumberofSteps + secondNumberOfSteps, stepCounter.get());
   }
   
   @Test
   public void testQueuedStepsSequentialWithMessageTools() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscription(drcSimulationTestHelper.getRos2Node(), FootstepStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(getSimpleRobotName()), (p) -> {
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

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      int firstNumberofSteps = 4;
      double stepX = 0.0;
      for (int currentStep = 0; currentStep < firstNumberofSteps; currentStep++)
      {
         
         ExecutionMode currentExecutionMode = ExecutionMode.QUEUE;
         if(currentStep == 0)
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
         FootstepDataListMessage footStepToSend = HumanoidMessageTools.createFootstepDataListMessage(currentFootStepList, swing, transfer, currentExecutionMode);

         side = side.getOppositeSide();

         drcSimulationTestHelper.publishToController(footStepToSend);

         stepX += stepLength;
         
         // send a new footstep when the current footstep is halfway done
         double simulationTime = (transfer + swing);

         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
         
      }

      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps - 1) - (((transfer + swing))*firstNumberofSteps - 1);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      assertEquals(firstNumberofSteps, stepCounter.get());
   }
   
   
   
   

   @Test
   public void testOnlyQueuedStepsWithTinySims() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);

      AtomicInteger stepCounter = new AtomicInteger();
      ROS2Tools.createCallbackSubscription(drcSimulationTestHelper.getRos2Node(), FootstepStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(getSimpleRobotName()), (p) -> {
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

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);


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

         drcSimulationTestHelper.publishToController(footMessage);
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);

         stepX += stepLength;
      }

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();


      double simulationTime = intitialTransfer - transfer + (transfer + swing) * (firstNumberofSteps);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

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
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
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
