package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCSteeringWheelEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspCylinderBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSteeringWheelRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCGraspCylinderBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private static final boolean DEBUG = false;

   @Before
   public void setUp()
   {
      if (NetworkPorts.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), getSimpleRobotName(), null, DRCObstacleCourseStartingLocation.DEFAULT,
            simulationTestingParameters, getRobotModel());
   }

   private static DRCSteeringWheelEnvironment createTestEnvironment()
   {
      ArrayList<Point3d> valveLocations = new ArrayList<Point3d>();
      LinkedHashMap<Point3d, Double> valveYawAngles_degrees = new LinkedHashMap<Point3d, Double>();
      LinkedHashMap<Point3d, Double> valvePitchAngles_degrees = new LinkedHashMap<Point3d, Double>();

      valveLocations.add(new Point3d(0.25, 0.0, 0.6));
      valveYawAngles_degrees.put(valveLocations.get(0), 0.0);
      valvePitchAngles_degrees.put(valveLocations.get(0), 90.0);

      return new DRCSteeringWheelEnvironment(valveLocations, valveYawAngles_degrees, valvePitchAngles_degrees);
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspSteeringWheelSpinner() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final GraspCylinderBehavior graspBehavior = new GraspCylinderBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), getRobotModel(), drcBehaviorTestHelper.getYoTime());

      ReferenceFrame chestFrame = drcBehaviorTestHelper.getReferenceFrames().getChestFrame();
      drcBehaviorTestHelper.updateRobotModel();

      RobotSide robotSideOfGraspingHand = RobotSide.LEFT;
      
      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableSteeringWheelRobot steeringWheel = (ContactableSteeringWheelRobot) testEnvironment.getEnvironmentRobots().get(0);
      RigidBodyTransform wheelTransformToWorld = new RigidBodyTransform();
      steeringWheel.getBodyTransformToWorld(wheelTransformToWorld);     
      TransformReferenceFrame steeringWheelFrame = new TransformReferenceFrame("steeringWheel", ReferenceFrame.getWorldFrame(), wheelTransformToWorld);
         
      
      FramePoint graspPoint = new FramePoint(steeringWheelFrame, 0.0, 0.0, steeringWheel.getValveRadius());
      FrameVector cylinderLongAxis = new FrameVector(chestFrame, 0.0, 0.0, 1.0);

      graspBehavior.initialize();
      graspBehavior.setGraspPose(robotSideOfGraspingHand, graspPoint, cylinderLongAxis, false);

      assertTrue(graspBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspBehavior);
      assertTrue(success);

      //TODO: Figure out how to test this
      assertTrue(graspBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
}
