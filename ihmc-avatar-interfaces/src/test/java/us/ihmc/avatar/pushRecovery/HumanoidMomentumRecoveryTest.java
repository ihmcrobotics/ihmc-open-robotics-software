package us.ihmc.avatar.pushRecovery;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

public abstract class HumanoidMomentumRecoveryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private PushRobotController pushController;

   private YoBoolean allowUpperBodyMomentumInSingleSupport;
   private YoBoolean allowUpperBodyMomentumInDoubleSupport;
   private YoBoolean allowUsingHighMomentumWeight;

   protected double getDoubleSupportPushMagnitude()
   {
      return 1000.0;
   }

   protected double getDoubleSupportPushDuration()
   {
      return 0.05;
   }

   protected double getSingleSupportPushMagnitude()
   {
      return 600.0;
   }

   protected double getSingleSupportPushDuration()
   {
      return 0.05;
   }
   
   protected double getXOffsetForSteps()
   {
      return 0.6;
   }

   protected double getYOffsetForSteps()
   {
      return 0.15;
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.6)
   @Test(timeout = 150000)
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupport() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();
      enableMomentum();

      assertTrue(standAndPush());
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.5)
   @Test(timeout = 110000)
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupportExpectFall() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();
      disableMomentum();

      assertFalse(standAndPush());
   }

   @ContinuousIntegrationTest(estimatedDuration = 34.3)
   @Test(timeout = 170000)
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraBackView();
      enableMomentum();

      assertTrue(stepAndPush());
   }

   @ContinuousIntegrationTest(estimatedDuration = 23.2)
   @Test(timeout = 120000)
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwingExpectFall() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraBackView();
      disableMomentum();

      assertFalse(stepAndPush());
   }

   @ContinuousIntegrationTest(estimatedDuration = 37.9)
   @Test(timeout = 190000)
   /**
    * End to end test that makes sure the momentum recovery does not get triggered during
    * some normal steps
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testRegularWalk() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      setupCameraSideView();
      enableMomentum();
      ControllerSpy controllerSpy = new ControllerSpy(drcSimulationTestHelper);

      double yOffset = getYOffsetForSteps();
      double xOffset = getXOffsetForSteps();
      
      FootstepDataListMessage message = new FootstepDataListMessage();
      addFootstep(new Point3D(xOffset / 2.0, yOffset, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3D(xOffset, -yOffset, -0.02), RobotSide.RIGHT, message);
      addFootstep(new Point3D(xOffset, yOffset * 2.0, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3D(xOffset, 0.0, -0.02), RobotSide.RIGHT, message);

      drcSimulationTestHelper.send(message);
      double simulationTime = 1.0 * message.footstepDataList.size() + 2.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      assertFalse(controllerSpy.wasMomentumTriggered());
      assertFalse(controllerSpy.wasWeightTriggered());
   }

   private void addFootstep(Point3D stepLocation, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());
      message.footstepDataList.add().set(footstepData);
   }

   private boolean standAndPush() throws SimulationExceededMaximumTimeException
   {
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      pushController.applyForce(new Vector3D(1.0, 0.0, 0.0), getDoubleSupportPushMagnitude(), getDoubleSupportPushDuration());
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
   }

   private boolean stepAndPush() throws SimulationExceededMaximumTimeException
   {
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(3.0, 0.3);
      FootstepDataMessage footstepData = new FootstepDataMessage();
      RobotSide stepSide = RobotSide.LEFT;

      double xOffset = getXOffsetForSteps();
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide);
      FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.3, 0.0, 0.0);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld);
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(stepSide.toByte());
      message.footstepDataList.add().set(footstepData);

      drcSimulationTestHelper.send(message);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push the robot
      pushController.applyForce(new Vector3D(0.0, -1.0, 0.0), getSingleSupportPushMagnitude(), getSingleSupportPushDuration());
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
   }

   private void enableMomentum()
   {
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);
   }

   private void disableMomentum()
   {
      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      Vector3D forcePointOffset = new Vector3D(0.0, 0.0, 0.1);
      pushController = new PushRobotController(drcSimulationTestHelper.getRobot(), drcSimulationTestHelper.getRobot().getRootJoint().getName(), forcePointOffset);

      allowUpperBodyMomentumInSingleSupport = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (YoBoolean) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");

      ThreadTools.sleep(1000);
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

   private class ControllerSpy extends SimpleRobotController
   {
      private final YoBoolean usingUpperBodyMomentum;
      private final YoBoolean usingHighMomentumWeight;

      private final YoBoolean momentumWasTriggered = new YoBoolean("momentumWasTriggered", registry);
      private final YoBoolean weightWasTriggered = new YoBoolean("weightWasTriggered", registry);

      public ControllerSpy(DRCSimulationTestHelper drcSimulationTestHelper)
      {
         usingUpperBodyMomentum = (YoBoolean) drcSimulationTestHelper.getYoVariable("usingUpperBodyMomentum");
         usingHighMomentumWeight = (YoBoolean) drcSimulationTestHelper.getYoVariable("usingHighMomentumWeight");
         drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
      }

      @Override
      public void doControl()
      {
         if (usingUpperBodyMomentum.getBooleanValue())
            momentumWasTriggered.set(true);
         if (usingHighMomentumWeight.getBooleanValue())
            weightWasTriggered.set(true);
      }

      public boolean wasWeightTriggered()
      {
         return weightWasTriggered.getBooleanValue();
      }

      public boolean wasMomentumTriggered()
      {
         return momentumWasTriggered.getBooleanValue();
      }

   }
}
