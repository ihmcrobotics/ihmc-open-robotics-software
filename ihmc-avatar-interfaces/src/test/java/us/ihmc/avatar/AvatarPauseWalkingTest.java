package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarPauseWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCRobotModel robotModel;
   private YoBoolean walkPaused;

   public abstract double getSwingTime();

   public abstract double getTransferTime();

   public abstract double getFinalTransferDuration();

   public abstract double getStepLength();

   public abstract double getStepWidth();

   public abstract double getTimeForPausing();

   public abstract double getTimeForResuming();

   public abstract int getNumberOfFootsteps();

   public abstract double getMaxICPPlanError();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testPauseWalking()
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(1.0));
      sendFootstepCommand(0.0, getNumberOfFootsteps());
      assertTrue(simulationTestHelper.simulateAndWait(getTimeForPausing()));
      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(simulationTestHelper.simulateAndWait(getTimeForResuming()));
      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @Test
   public void testTwoIndependentSteps()
   {
      setupTest();
      assertTrue(simulationTestHelper.simulateAndWait(0.5));

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(),
                                                                                                   getTransferTime(),
                                                                                                   getFinalTransferDuration());
      FramePoint3D location = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT), 0.1, 0.0, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, new Quaternion()));
      simulationTestHelper.publishToController(footstepMessage);

      double simulationTime = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + getSwingTime() + getFinalTransferDuration()
            + 0.5;
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("walkingCurrentState");
      assertEquals(WalkingStateEnum.STANDING, walkingState.getEnumValue());

      footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(), getTransferTime());
      location = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT), 0.2, 0.0, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, new Quaternion()));
      simulationTestHelper.publishToController(footstepMessage);

      assertTrue(simulationTestHelper.simulateAndWait(3.0));
   }

   @Test
   public void testStartSecondStepWhileTransitioningToStand()
   {
      setupTest();
      assertTrue(simulationTestHelper.simulateAndWait(0.5));

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(), getTransferTime());
      FramePoint3D location = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT), 0.1, 0.0, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, new Quaternion()));
      simulationTestHelper.publishToController(footstepMessage);

      double simulationTime = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + getSwingTime()
            + 0.5 * getFinalTransferDuration();
      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));

      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("walkingCurrentState");
      assertEquals(WalkingStateEnum.TO_STANDING, walkingState.getEnumValue());

      footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(), getTransferTime());
      location = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.RIGHT), 0.2, 0.0, 0.0);
      location.changeFrame(ReferenceFrame.getWorldFrame());
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, new Quaternion()));
      simulationTestHelper.publishToController(footstepMessage);

      assertTrue(simulationTestHelper.simulateAndWait(3.0));
   }

   @Test
   public void testPauseWalkingForward()
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(1.0));
      sendFootstepCommand(getStepLength(), getNumberOfFootsteps());
      assertTrue(simulationTestHelper.simulateAndWait(getTimeForPausing()));
      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(simulationTestHelper.simulateAndWait(getTimeForResuming()));
      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @Test
   public void testPauseWalkingInitialTransfer()
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(1.0));

      sendFootstepCommand(0.0, getNumberOfFootsteps());
      assertTrue(simulationTestHelper.simulateAndWait(0.85));

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);

      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(simulationTestHelper.simulateAndWait(2.0));

      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   @Test
   public void testPauseWalkingInitialTransferOneStep()
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(1.0));

      FramePoint3D stepLocation = new FramePoint3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      stepLocation.changeFrame(ReferenceFrame.getWorldFrame());

      double swingTime = 1.0;

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(swingTime, getTransferTime());
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, stepLocation, new Quaternion()));
      footstepMessage.setFinalTransferDuration(getFinalTransferDuration());

      simulationTestHelper.publishToController(footstepMessage);

      assertTrue(simulationTestHelper.simulateAndWait(0.85));

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);

      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(simulationTestHelper.simulateAndWait(2.0));

      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(swingTime + getTransferTime() + getFinalTransferDuration()));
   }

   @Test
   public void testPauseWalkingForwardInitialTransfer()
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(1.0));

      sendFootstepCommand(getStepLength(), getNumberOfFootsteps());
      assertTrue(simulationTestHelper.simulateAndWait(1.0));

      PauseWalkingMessage pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(true);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(true);

      assertTrue(simulationTestHelper.simulateAndWait(2.0));

      pauseWalkingMessage = HumanoidMessageTools.createPauseWalkingMessage(false);
      simulationTestHelper.publishToController(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(simulationTestHelper.simulateAndWait(getNumberOfFootsteps() * (getSwingTime() + getTransferTime())));
   }

   private void sendFootstepCommand(double stepLength, int numberOfFootsteps)
   {
      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(getSwingTime(), getTransferTime());
      RobotSide side = RobotSide.LEFT;
      Quaternion orientation = new Quaternion();
      for (int i = 1; i < numberOfFootsteps; i++)
      {
         addFootstep(new Point3D(i * stepLength, side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side, footstepMessage);
         side = side.getOppositeSide();
      }
      addFootstep(new Point3D((numberOfFootsteps - 1) * stepLength, side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side, footstepMessage);
      footstepMessage.setFinalTransferDuration(getFinalTransferDuration());
      simulationTestHelper.publishToController(footstepMessage);
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(orient);
      footstepData.setRobotSide(robotSide.toByte());
      message.getFootstepDataList().add().set(footstepData);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();

      robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, emptyEnvironment, simulationTestingParameters);
      simulationTestHelper.addDesiredICPContinuityAssertion(getMaxICPPlanError());
      simulationTestHelper.start();
      YoRegistry registry = simulationTestHelper.getRootRegistry();
      walkPaused = new YoBoolean("isWalkPaused", registry);
      ThreadTools.sleep(1000);
      setupCameraSideView();
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

}
