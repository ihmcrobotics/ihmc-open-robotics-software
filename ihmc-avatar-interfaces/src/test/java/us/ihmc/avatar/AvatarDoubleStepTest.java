package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
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
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public abstract class AvatarDoubleStepTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;

   public double getMaxICPPlanError()
   {
      return 0.02;
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
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

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testTwoStepsInARowSameSide() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupPlanContinuityTesters();

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

   @ContinuousIntegrationTest(estimatedDuration = 76.4, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 100000)
   public void testTwoStepsInARowSameSideAfterFirstSep() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupPlanContinuityTesters();

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


   @ContinuousIntegrationTest(estimatedDuration = 76.4, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 100000)
   public void testTwoStepsInARowLongTransferSameSide() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupPlanContinuityTesters();

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

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   public void testTwoStepsStandingInBetween() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupPlanContinuityTesters();

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
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      ThreadTools.sleep(1000);
      setupCameraSideView();
   }

   private void setupPlanContinuityTesters()
   {
      final YoDouble desiredICPX = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPX");
      final YoDouble desiredICPY = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPY");

      final MutableDouble previousDesiredICPX = new MutableDouble();
      final MutableDouble previousDesiredICPY = new MutableDouble();

      final MutableBoolean xInitialized = new MutableBoolean(false);
      final MutableBoolean yInitialized = new MutableBoolean(false);

      desiredICPX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (xInitialized.getValue())
            {
               assertTrue("ICP plan desired X jumped from " + previousDesiredICPX.getValue() + " to " + desiredICPX.getDoubleValue() + " in one control DT.",
                          Math.abs(desiredICPX.getDoubleValue() - previousDesiredICPX.getValue()) < getMaxICPPlanError());
            }
            else
            {
               xInitialized.setValue(true);
            }
            previousDesiredICPX.setValue(desiredICPX.getDoubleValue());

         }
      });

      desiredICPY.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (yInitialized.getValue())
            {
               assertTrue("ICP plan desired Y jumped from " + previousDesiredICPY.getValue() + " to " + desiredICPY.getDoubleValue() + " in one control DT.",
                          Math.abs(desiredICPY.getDoubleValue() - previousDesiredICPY.getValue()) < getMaxICPPlanError());
            }
            else
            {
               yInitialized.setValue(true);
            }
            previousDesiredICPY.setValue(desiredICPY.getDoubleValue());

         }
      });
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
