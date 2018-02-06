package us.ihmc.avatar.straightLegWalking;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.StairsUpAndDownEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public abstract class AvatarStraightLegSingleStepTest implements MultiRobotTestInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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
      footstepDataListMessage.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, stepWidth / 2.0, 0.0), new FrameQuaternion()));
      footstepDataListMessage.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -stepWidth / 2.0, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.send(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testWideStep(double stepWidth, double stanceWidth) throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(0.0, -stepWidth, 0.0), new FrameQuaternion()));
      footstepDataListMessage.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(0.0, stanceWidth - stepWidth, 0.0), new FrameQuaternion()));

      drcSimulationTestHelper.send(footstepDataListMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));

      Point3D center = new Point3D(0.0, 0.5 * stanceWidth - stepWidth, 1.05);
      Vector3D plusMinusVector = new Vector3D(0.1, stanceWidth / 2.0, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testSteppingDown(double stepDownHeight, double stepLength, double stanceWidth) throws SimulationExceededMaximumTimeException
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

      message.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, dropHeight), new Quaternion()));

      drcSimulationTestHelper.send(message);

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

      message.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, dropHeight), new Quaternion()));
      message.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -0.5 * stanceWidth, dropHeight), new Quaternion()));

      drcSimulationTestHelper.send(message);

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
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
