package us.ihmc.avatar.pushRecovery;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;

@Tag("humanoid-push-recovery")
public abstract class HumanoidMomentumRecoveryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private PushRobotControllerSCS2 pushController;

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

   @Test
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupport()
   {
      setupTest();
      setupCameraSideView();
      enableMomentum();

      assertTrue(standAndPush());
   }

   @Test
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringDoubleSupportExpectFall()
   {
      setupTest();
      setupCameraSideView();
      disableMomentum();

      assertFalse(standAndPush());
   }

   @Test
   /**
    * End to end test that makes sure the robot can recover from a push using upper body momentum
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwing()
   {
      setupTest();
      setupCameraBackView();
      enableMomentum();

      assertTrue(stepAndPush());
   }

   @Test
   /**
    * End to end test that makes sure the robot falls during test if momentum is disabled
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testPushDuringSwingExpectFall()
   {
      setupTest();
      setupCameraBackView();
      disableMomentum();

      assertFalse(stepAndPush());
   }

   @Test
   /**
    * End to end test that makes sure the momentum recovery does not get triggered during some normal
    * steps
    *
    * @throws SimulationExceededMaximumTimeException
    */
   public void testRegularWalk()
   {
      setupTest();
      setupCameraSideView();
      enableMomentum();
      ControllerSpy controllerSpy = new ControllerSpy(simulationTestHelper);

      double yOffset = getYOffsetForSteps();
      double xOffset = getXOffsetForSteps();

      FootstepDataListMessage message = new FootstepDataListMessage();
      addFootstep(new Point3D(xOffset / 2.0, yOffset, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3D(xOffset, -yOffset, -0.02), RobotSide.RIGHT, message);
      addFootstep(new Point3D(xOffset, yOffset * 2.0, -0.02), RobotSide.LEFT, message);
      addFootstep(new Point3D(xOffset, 0.0, -0.02), RobotSide.RIGHT, message);

      simulationTestHelper.publishToController(message);
      double simulationTime = 1.0 * message.getFootstepDataList().size() + 2.0;
      assertTrue(simulationTestHelper.simulateNow(simulationTime));

      assertFalse(controllerSpy.wasMomentumTriggered());
      assertFalse(controllerSpy.wasWeightTriggered());
   }

   private void addFootstep(Point3D stepLocation, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());
      message.getFootstepDataList().add().set(footstepData);
   }

   private boolean standAndPush()
   {
      assertTrue(simulationTestHelper.simulateNow(1.0));
      pushController.applyForce(new Vector3D(1.0, 0.0, 0.0), getDoubleSupportPushMagnitude(), getDoubleSupportPushDuration());
      return simulationTestHelper.simulateNow(10.0);
   }

   private boolean stepAndPush()
   {
      assertTrue(simulationTestHelper.simulateNow(1.0));

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(3.0, 0.3);
      FootstepDataMessage footstepData = new FootstepDataMessage();
      RobotSide stepSide = RobotSide.LEFT;

      ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(stepSide);
      FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.3, 0.0, 0.0);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(stepSide.toByte());
      message.getFootstepDataList().add().set(footstepData);

      simulationTestHelper.publishToController(message);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      // push the robot
      pushController.applyForce(new Vector3D(0.0, -1.0, 0.0), getSingleSupportPushMagnitude(), getSingleSupportPushDuration());
      return simulationTestHelper.simulateNow(5.0);
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
      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, emptyEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      Vector3D forcePointOffset = new Vector3D(0.0, 0.0, 0.1);
      pushController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationSession().getTime(),
                                                   simulationTestHelper.getRobot(),
                                                   simulationTestHelper.getRobot().getFloatingRootJoint().getName(),
                                                   forcePointOffset);

      allowUpperBodyMomentumInSingleSupport = (YoBoolean) simulationTestHelper.findVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (YoBoolean) simulationTestHelper.findVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (YoBoolean) simulationTestHelper.findVariable("allowUsingHighMomentumWeight");

      ThreadTools.sleep(1000);
   }

   private void setupCameraBackView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
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
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
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

      public ControllerSpy(SCS2AvatarTestingSimulation simulationTestHelper)
      {
         usingUpperBodyMomentum = (YoBoolean) simulationTestHelper.findVariable("usingUpperBodyMomentum");
         usingHighMomentumWeight = (YoBoolean) simulationTestHelper.findVariable("usingHighMomentumWeight");
         simulationTestHelper.addRobotControllerOnControllerThread(this);
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
