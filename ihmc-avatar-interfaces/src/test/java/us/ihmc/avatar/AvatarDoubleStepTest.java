package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarDoubleStepTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;

   public double getMaxICPPlanError()
   {
      return 0.02;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      System.out.println("blah? " + simulationTestingParameters.getKeepSCSUp());
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
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Tag("humanoid-flat-ground-2")
   @Test
   public void testTwoStepsInARowSameSide() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide stepSide = RobotSide.LEFT;
      FramePoint3D stepLocation = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide));
      stepLocation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(1.0, 2.0);
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation, new Quaternion()));
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation, new Quaternion()));
      footstepMessage.setFinalTransferDuration(2.0);

      drcSimulationTestHelper.publishToController(footstepMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.0));
   }

   @Tag("allocation-slow")
   @Test
   public void testTwoStepsInARowSameSideAfterFirstSep() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide stepSide = RobotSide.RIGHT;
      FramePoint3D stepLocation1 = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide.getOppositeSide()));
      FramePoint3D stepLocation2 = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide));
      FramePoint3D stepLocation3 = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide.getOppositeSide()));
      stepLocation1.changeFrame(ReferenceFrame.getWorldFrame());
      stepLocation2.changeFrame(ReferenceFrame.getWorldFrame());
      stepLocation1.addX(0.2);
      stepLocation2.addX(0.4);
      stepLocation3.addX(0.6);


      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(1.0, 2.0);
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide.getOppositeSide(), stepLocation1, new Quaternion()));
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation2, new Quaternion()));
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation2, new Quaternion()));
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide.getOppositeSide(), stepLocation3, new Quaternion()));
      footstepMessage.setFinalTransferDuration(2.0);

      drcSimulationTestHelper.publishToController(footstepMessage);

      double stepDuration = 3.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4 * stepDuration + 3.0));
   }

   @Tag("allocation-slow")
   @Test
   public void testTwoStepsInARowLongTransferSameSide() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide stepSide = RobotSide.LEFT;
      FramePoint3D stepLocation = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide));
      stepLocation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(1.0, 20.0);
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation, new Quaternion()));
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation, new Quaternion()));
      footstepMessage.setFinalTransferDuration(2.0);

      drcSimulationTestHelper.publishToController(footstepMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.0));
   }

   @Tag("humanoid-flat-ground-2")
   @Test
   public void testTwoStepsStandingInBetween() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide stepSide = RobotSide.LEFT;
      FramePoint3D stepLocation = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide));
      stepLocation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(1.0, 2.0);
      footstepMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(stepSide, stepLocation, new Quaternion()));
      footstepMessage.setFinalTransferDuration(2.0);

      drcSimulationTestHelper.publishToController(footstepMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) drcSimulationTestHelper.getYoVariable("walkingCurrentState");
      assertEquals("Robot isn't yet standing.", WalkingStateEnum.STANDING, walkingState.getEnumValue());

      drcSimulationTestHelper.publishToController(footstepMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
         }
      };
      robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setCheckForDesiredICPContinuity(true, getMaxICPPlanError());
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      ThreadTools.sleep(1000);
      setupCameraSideView();
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
