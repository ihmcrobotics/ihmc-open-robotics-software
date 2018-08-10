package us.ihmc.avatar.straightLegWalking;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarStraightLegSingleStepTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testForwardStep(double stepLength, double stepWidth) throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, stepWidth / 2.0, 0.0), new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testForwardStepWithPause(double stepLength, double stepWidth) throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      footstepDataListMessage.getFootstepDataList().clear();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testForwardSteps(double startingLength, double nominalLength, int stepsToLength, double stepWidth, int totalSteps) throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();

      TDoubleArrayList stepLengths = new TDoubleArrayList();
      stepsToLength++;

      double length = startingLength;
      stepLengths.add(length);
      for (int i = 1; i < Math.max(stepsToLength, totalSteps); i++)
      {
         double alpha = Math.min((double) i / (double) stepsToLength, 1.0);
         length = stepLengths.get(i - 1) + InterpolationTools.linearInterpolate(startingLength, nominalLength, alpha);
         stepLengths.add(length);
      }
      stepLengths.add(length);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      double width = stepWidth;
      RobotSide side = RobotSide.LEFT;
      for (int i = 0; i < stepLengths.size(); i++)
      {
         FootstepDataMessage message = footstepDataListMessage.getFootstepDataList().add();
         message.set(HumanoidMessageTools.createFootstepDataMessage(side, new Point3D(stepLengths.get(i), width / 2.0, 0.0), new FrameQuaternion()));
         width = -width;
         side = side.getOppositeSide();

//         message.setSwingDuration(0.5);
      }

//      footstepDataListMessage.setAreFootstepsAdjustable(true);

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0));

      Point3D center = new Point3D(stepLengths.get(stepLengths.size() - 1), 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testWideStep(double stepWidth, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Point3D step1 = new Point3D(0.0, -stepWidth, 0.0);
      Point3D step2 = new Point3D(0.0, stanceWidth - stepWidth, 0.0);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step1, new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step2, new FrameQuaternion()));

      drcSimulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      Point3D center = new Point3D();
      center.interpolate(step1, step2, 0.5);
      center.addZ(1.0);
      Vector3D plusMinusVector = new Vector3D(0.1, stanceWidth / 2.0, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testSteppingDown(double stepDownHeight, double stepLength, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      testSteppingDown(stepDownHeight, stepDownHeight, stepLength, stanceWidth);
   }

   public void testSteppingDown(double stepDownHeight, double stepHeight, double stepLength, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, -stepDownHeight), new Quaternion()));

      drcSimulationTestHelper.publishToController(message);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testSteppingDownWithClosing(double stepDownHeight, double stepLength, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepDownHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, dropHeight), new Quaternion()));
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -0.5 * stanceWidth, dropHeight), new Quaternion()));

      drcSimulationTestHelper.publishToController(message);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      setupCamera();
      ThreadTools.sleep(1000);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
