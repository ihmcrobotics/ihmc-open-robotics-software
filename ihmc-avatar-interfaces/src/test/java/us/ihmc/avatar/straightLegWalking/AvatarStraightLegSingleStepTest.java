package us.ihmc.avatar.straightLegWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarStraightLegSingleStepTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private Double stepLength = null;
   private Double stepWidth = null;
   private Double stanceWidth = null;
   private Double stepDownHeight = null;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      stepLength = null;
      stepWidth = null;
      stanceWidth = null;
      stepDownHeight = null;
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      stepLength = null;
      stepWidth = null;
      stanceWidth = null;
      stepDownHeight = null;
   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;
   }

   public void setStepWidth(double stepWidth)
   {
      this.stepWidth = stepWidth;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public void setStepDownHeight(double stepDownHeight)
   {
      this.stepDownHeight = stepDownHeight;
   }

   @Test
   public void testForwardStep()
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                 new Point3D(stepLength, stepWidth / 2.0, 0.0),
                                                                                 new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                 new Point3D(stepLength, -stepWidth / 2.0, 0.0),
                                                                                 new FrameQuaternion()));

      simulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(simulationTestHelper.simulateNow(4.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testForwardStepWithPause()
   {
      setupTest();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                 new Point3D(stepLength, stepWidth / 2.0, 0.0),
                                                                                 new FrameQuaternion()));

      simulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(simulationTestHelper.simulateNow(3.0));

      footstepDataListMessage.getFootstepDataList().clear();
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                 new Point3D(stepLength, -stepWidth / 2.0, 0.0),
                                                                                 new FrameQuaternion()));

      simulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(simulationTestHelper.simulateNow(3.0));

      Point3D center = new Point3D(stepLength, 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testForwardSteps()
   {
      double startingLength = 0.4;
      double nominalLength = 0.75;
      double stepWidth = 0.2;
      int stepsToLength = 3;
      int totalSteps = 20;

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

      simulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(simulationTestHelper.simulateNow(20.0));

      Point3D center = new Point3D(stepLengths.get(stepLengths.size() - 1), 0.0, 0.9);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.15);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testWideStep()
   {
      setupTest();

      Point3D step1 = new Point3D(0.0, -stepWidth, 0.0);
      Point3D step2 = new Point3D(0.0, stanceWidth - stepWidth, 0.0);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, step1, new FrameQuaternion()));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, step2, new FrameQuaternion()));

      simulationTestHelper.publishToController(footstepDataListMessage);

      assertTrue(simulationTestHelper.simulateNow(6.0));

      Point3D center = new Point3D();
      center.interpolate(step1, step2, 0.5);
      center.addZ(1.0);
      Vector3D plusMinusVector = new Vector3D(0.1, stanceWidth / 2.0, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testSteppingDown()
   {
      runSteppingDown(stepDownHeight, stepDownHeight, stepLength, stanceWidth);
   }

   public void runSteppingDown(double stepDownHeight, double stepHeight, double stepLength, double stanceWidth)
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), stepDownEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(simulationTestHelper.simulateNow(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add()
             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                 new Point3D(stepLength, 0.5 * stanceWidth, -stepDownHeight),
                                                                 new Quaternion()));

      simulationTestHelper.publishToController(message);

      assertTrue(simulationTestHelper.simulateNow(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSteppingDownWithClosing()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double dropHeight = -stepDownHeight;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      stepHeights.add(dropHeight);
      stepLengths.add(stepLength);

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, dropHeight);
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), stepDownEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

      setupCamera();

      ThreadTools.sleep(1000);

      assertTrue(simulationTestHelper.simulateNow(1.0));

      FootstepDataListMessage message = new FootstepDataListMessage();

      message.getFootstepDataList().add()
             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(stepLength, 0.5 * stanceWidth, dropHeight), new Quaternion()));
      message.getFootstepDataList().add()
             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(stepLength, -0.5 * stanceWidth, dropHeight), new Quaternion()));

      simulationTestHelper.publishToController(message);

      assertTrue(simulationTestHelper.simulateNow(6.0));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupTest()
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();

      setupCamera();
      ThreadTools.sleep(1000);

      assertTrue(simulationTestHelper.simulateNow(1.0));
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }
}
