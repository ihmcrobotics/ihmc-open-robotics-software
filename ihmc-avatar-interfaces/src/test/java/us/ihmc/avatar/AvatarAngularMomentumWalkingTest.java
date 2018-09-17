package us.ihmc.avatar;

import controller_msgs.msg.dds.*;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public abstract class AvatarAngularMomentumWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;

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

   private void setupTest()
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();

      ThreadTools.sleep(1000);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalk() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkWithCorruptedMomentum() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));
      drcSimulationTestHelper.publishToController(createCorruptedMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkTransferDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkTransferBigDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkSwingDelayedMomentum() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialTransfer);
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessage(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }


   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkZeroMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessageZeroMomentumFirstStep(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testForwardWalkNoMomentumFirstStep() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      YoBoolean planSwingAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanSwingAngularMomentumWithCommand");
      YoBoolean planTransferAngularMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("PlanTransferAngularMomentumWithCommand");
      planSwingAngularMomentum.set(true);
      planTransferAngularMomentum.set(true);

      drcSimulationTestHelper.publishToController(createFootstepMessage(numberOfSteps));
      drcSimulationTestHelper.publishToController(createMomentumTrajectoryMessageNoMomentumFirstStep(initialTransfer, transfer, swing, numberOfSteps + 1));

      double simulationTime = initialTransfer + (transfer + swing) * (numberOfSteps + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   private FootstepDataListMessage createFootstepMessage(int numberOfSteps)
   {
      RobotSide side = RobotSide.LEFT;
      double stepLength = 0.4;
      double stepWidth = 0.25;
      FootstepDataListMessage message = new FootstepDataListMessage();

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         Point3D footLocation = new Point3D(stepLength * (currentStep + 1), side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));
         side = side.getOppositeSide();
      }

      Point3D footLocation = new Point3D(stepLength * (numberOfSteps), side.negateIfRightSide(stepWidth / 2), 0.0);
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, footLocation, footOrientation));

      return message;
   }

   private MomentumTrajectoryMessage createMomentumTrajectoryMessage(double initialTransferDuration, double transferDuration, double swingDuration,
                                                                     int numberOfSteps)
   {
      MomentumTrajectoryMessage momentumTrajectoryMessage = new MomentumTrajectoryMessage();
      EuclideanTrajectoryMessage angularMomentum = momentumTrajectoryMessage.getAngularMomentumTrajectory();

      double dt = 0.001;
      for (double time = 0; time <= initialTransferDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(time);
         point.getPosition().setToZero();
         point.getLinearVelocity().setToZero();
      }

      RobotSide robotSide = RobotSide.LEFT;
      double currentTime = initialTransferDuration;
      double angularMomentumXMagnitude = -3.0;
      double angularMomentumYMagnitude = -10.0;
      double angularMomentumXFrequency = 2.0 * Math.PI / swingDuration;
      double angularMomentumYFrequency = 2.0 * Math.PI / (2.0 * swingDuration);
      for (double time = 0; time <= swingDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(currentTime + time);

         double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
         double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
         yMomentum = robotSide.negateIfRightSide(yMomentum);

         point.getPosition().set(xMomentum, yMomentum, 0.0);
         point.getLinearVelocity().setToZero();
      }

      currentTime += swingDuration;

      for (int stepNumber = 1; stepNumber < numberOfSteps; stepNumber++)
      {
         for (double time = 0; time < transferDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);
            point.getPosition().setToZero();
            point.getLinearVelocity().setToZero();
         }

         currentTime += transferDuration;

         for (double time = 0; time < swingDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);

            double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
            double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
            yMomentum = robotSide.negateIfRightSide(yMomentum);

            point.getPosition().set(xMomentum, yMomentum, 0.0);
            point.getLinearVelocity().setToZero();
         }

         currentTime += swingDuration;
      }

      return momentumTrajectoryMessage;
   }

   private MomentumTrajectoryMessage createCorruptedMomentumTrajectoryMessage(double initialTransferDuration, double transferDuration, double swingDuration,
                                                                     int numberOfSteps)
   {
      MomentumTrajectoryMessage momentumTrajectoryMessage = new MomentumTrajectoryMessage();
      EuclideanTrajectoryMessage angularMomentum = momentumTrajectoryMessage.getAngularMomentumTrajectory();

      double dt = 0.001;
      for (double time = 0; time <= initialTransferDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(time);
         point.getPosition().setToZero();
         point.getLinearVelocity().setToZero();
      }


      RobotSide robotSide = RobotSide.LEFT;
      double currentTime = initialTransferDuration;
      double angularMomentumXMagnitude = -3.0;
      double angularMomentumYMagnitude = -10.0;
      double angularMomentumXFrequency = 2.0 * Math.PI / swingDuration;
      double angularMomentumYFrequency = 2.0 * Math.PI / (2.0 * swingDuration);
      for (double time = 0; time <= swingDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(currentTime + time);

         double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
         double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
         yMomentum = robotSide.negateIfRightSide(yMomentum);

         point.getPosition().set(xMomentum, yMomentum, 0.0);
         point.getLinearVelocity().setToZero();
      }

      angularMomentum.getTaskspaceTrajectoryPoints().getLast().getPosition().setToNaN();
      angularMomentum.getTaskspaceTrajectoryPoints().getLast().getLinearVelocity().setToNaN();

      currentTime += swingDuration;

      for (int stepNumber = 1; stepNumber < numberOfSteps; stepNumber++)
      {
         for (double time = 0; time < transferDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);
            point.getPosition().setToZero();
            point.getLinearVelocity().setToZero();
         }

         currentTime += transferDuration;

         for (double time = 0; time < swingDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);

            double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
            double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
            yMomentum = robotSide.negateIfRightSide(yMomentum);

            point.getPosition().set(xMomentum, yMomentum, 0.0);
            point.getLinearVelocity().setToZero();
         }

         currentTime += swingDuration;
      }

      return momentumTrajectoryMessage;
   }

   private MomentumTrajectoryMessage createMomentumTrajectoryMessageZeroMomentumFirstStep(double initialTransferDuration, double transferDuration, double swingDuration,
                                                                     int numberOfSteps)
   {
      MomentumTrajectoryMessage momentumTrajectoryMessage = new MomentumTrajectoryMessage();
      EuclideanTrajectoryMessage angularMomentum = momentumTrajectoryMessage.getAngularMomentumTrajectory();

      double dt = 0.001;
      for (double time = 0; time <= initialTransferDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(time);
         point.getPosition().setToZero();
         point.getLinearVelocity().setToZero();
      }

      RobotSide robotSide = RobotSide.LEFT;
      double currentTime = initialTransferDuration;
      double angularMomentumXMagnitude = -3.0;
      double angularMomentumYMagnitude = -10.0;
      double angularMomentumXFrequency = 2.0 * Math.PI / swingDuration;
      double angularMomentumYFrequency = 2.0 * Math.PI / (2.0 * swingDuration);
      for (double time = 0; time <= swingDuration; time += dt)
      {
         EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
         point.setTime(currentTime + time);

         point.getPosition().setToZero();
         point.getLinearVelocity().setToZero();
      }

      currentTime += swingDuration;

      for (int stepNumber = 1; stepNumber < numberOfSteps; stepNumber++)
      {
         for (double time = 0; time < transferDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);
            point.getPosition().setToZero();
            point.getLinearVelocity().setToZero();
         }

         currentTime += transferDuration;

         for (double time = 0; time < swingDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);

            double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
            double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
            yMomentum = robotSide.negateIfRightSide(yMomentum);

            point.getPosition().set(xMomentum, yMomentum, 0.0);
            point.getLinearVelocity().setToZero();
         }

         currentTime += swingDuration;
      }

      return momentumTrajectoryMessage;
   }

   private MomentumTrajectoryMessage createMomentumTrajectoryMessageNoMomentumFirstStep(double initialTransferDuration, double transferDuration, double swingDuration,
                                                                                          int numberOfSteps)
   {
      MomentumTrajectoryMessage momentumTrajectoryMessage = new MomentumTrajectoryMessage();
      EuclideanTrajectoryMessage angularMomentum = momentumTrajectoryMessage.getAngularMomentumTrajectory();

      double dt = 0.001;


      RobotSide robotSide = RobotSide.LEFT;
      double angularMomentumXMagnitude = -3.0;
      double angularMomentumYMagnitude = -10.0;
      double angularMomentumXFrequency = 2.0 * Math.PI / swingDuration;
      double angularMomentumYFrequency = 2.0 * Math.PI / (2.0 * swingDuration);

      double currentTime = initialTransferDuration + swingDuration;

      for (int stepNumber = 1; stepNumber < numberOfSteps; stepNumber++)
      {
         for (double time = 0; time < transferDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);
            point.getPosition().setToZero();
            point.getLinearVelocity().setToZero();
         }

         currentTime += transferDuration;

         for (double time = 0; time < swingDuration; time += dt)
         {
            EuclideanTrajectoryPointMessage point = angularMomentum.getTaskspaceTrajectoryPoints().add();
            point.setTime(currentTime + time);

            double xMomentum = angularMomentumXMagnitude * Math.sin(angularMomentumXFrequency * time);
            double yMomentum = angularMomentumYMagnitude * Math.sin(angularMomentumYFrequency * time);
            yMomentum = robotSide.negateIfRightSide(yMomentum);

            point.getPosition().set(xMomentum, yMomentum, 0.0);
            point.getLinearVelocity().setToZero();
         }

         currentTime += swingDuration;
      }

      return momentumTrajectoryMessage;
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
