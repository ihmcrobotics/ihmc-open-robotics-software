package us.ihmc.avatar.pushRecovery;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HumanoidMomentumRecoveryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3d(0.0, 0.0, 0.0), 0.0);
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private PushRobotController pushController;

   private BooleanYoVariable allowUpperBodyMomentumInSingleSupport;
   private BooleanYoVariable allowUpperBodyMomentumInDoubleSupport;
   private BooleanYoVariable allowUsingHighMomentumWeight;

   private static final double doubleSupportPushMagnitude = 1000.0;
   private static final double doubleSupportPushDuration = 0.05;

   private static final double singleSupportPushMagnitude = 600.0;
   private static final double singleSupportPushDuration = 0.05;

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

      FootstepDataListMessage message = new FootstepDataListMessage();
      addFootstep(new Point3d(0.3, 0.15, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3d(0.6, -0.15, -0.02), RobotSide.RIGHT, message);
      addFootstep(new Point3d(0.6, 0.3, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3d(0.6, 0.0, -0.02), RobotSide.RIGHT, message);

      drcSimulationTestHelper.send(message);
      double simulationTime = 1.0 * message.footstepDataList.size() + 2.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      assertFalse(controllerSpy.wasMomentumTriggered());
      assertFalse(controllerSpy.wasWeightTriggered());
   }

   private void addFootstep(Point3d stepLocation, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      message.add(footstepData);
   }

   private boolean standAndPush() throws SimulationExceededMaximumTimeException
   {
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      pushController.applyForce(new Vector3d(1.0, 0.0, 0.0), doubleSupportPushMagnitude, doubleSupportPushDuration);
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
   }

   private boolean stepAndPush() throws SimulationExceededMaximumTimeException
   {
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage(3.0, 0.3);
      FootstepDataMessage footstepData = new FootstepDataMessage();
      RobotSide stepSide = RobotSide.LEFT;

      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide);
      FramePoint placeToStepInWorld = new FramePoint(soleFrame, 0.3, 0.0, 0.0);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld.getPointCopy());
      footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(stepSide);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      message.add(footstepData);

      drcSimulationTestHelper.send(message);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push the robot
      pushController.applyForce(new Vector3d(0.0, -1.0, 0.0), singleSupportPushMagnitude, singleSupportPushDuration);
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
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return location;
         }
      };
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(emptyEnvironment, className, startingLocation, simulationTestingParameters, robotModel);
      Vector3d forcePointOffset = new Vector3d(0.0, 0.0, 0.1);
      pushController = new PushRobotController(drcSimulationTestHelper.getRobot(), drcSimulationTestHelper.getRobot().getRootJoint().getName(), forcePointOffset);

      allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");

      ThreadTools.sleep(1000);
   }

   private void setupCameraBackView()
   {
      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraSideView()
   {
      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(0.0, 10.0, 1.0);
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
      private final BooleanYoVariable usingUpperBodyMomentum;
      private final BooleanYoVariable usingHighMomentumWeight;

      private final BooleanYoVariable momentumWasTriggered = new BooleanYoVariable("momentumWasTriggered", registry);
      private final BooleanYoVariable weightWasTriggered = new BooleanYoVariable("weightWasTriggered", registry);

      public ControllerSpy(DRCSimulationTestHelper drcSimulationTestHelper)
      {
         usingUpperBodyMomentum = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("usingUpperBodyMomentum");
         usingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("usingHighMomentumWeight");
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
