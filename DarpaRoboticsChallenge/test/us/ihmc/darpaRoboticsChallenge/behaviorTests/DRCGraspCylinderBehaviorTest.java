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

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCGraspCylinderEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspCylinderBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableStaticCylinderRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
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
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), getSimpleRobotName(), null, DRCObstacleCourseStartingLocation.DEFAULT,
            simulationTestingParameters, getRobotModel());
   }

   private static DRCGraspCylinderEnvironment createTestEnvironment()
   {
      ArrayList<Point3d> cylinderOrigins = new ArrayList<Point3d>();
      LinkedHashMap<Point3d, Double> cylinderYawAngles_degrees = new LinkedHashMap<Point3d, Double>();
      LinkedHashMap<Point3d, Double> cylinderPitchAngles_degrees = new LinkedHashMap<Point3d, Double>();
      LinkedHashMap<Point3d, Double> cylinderRollAngles_degrees = new LinkedHashMap<Point3d, Double>();

      // X-axis cylinder
      cylinderOrigins.add(new Point3d(0.5, 0.0, 0.6));
      cylinderYawAngles_degrees.put(cylinderOrigins.get(0), 0.0);
      cylinderPitchAngles_degrees.put(cylinderOrigins.get(0), 90.0);
      cylinderRollAngles_degrees.put(cylinderOrigins.get(0), 0.0);

      // Y-axis cylinder
      cylinderOrigins.add(new Point3d(0.7, 0.3, 0.6));
      cylinderYawAngles_degrees.put(cylinderOrigins.get(1), 0.0);
      cylinderPitchAngles_degrees.put(cylinderOrigins.get(1), 0.0);
      cylinderRollAngles_degrees.put(cylinderOrigins.get(1), 90.0);

      // Z-axis cylinder
      cylinderOrigins.add(new Point3d(0.25, 0.0, 0.7));
      cylinderYawAngles_degrees.put(cylinderOrigins.get(2), 0.0);
      cylinderPitchAngles_degrees.put(cylinderOrigins.get(2), 0.0);
      cylinderRollAngles_degrees.put(cylinderOrigins.get(2), 0.0);

      return new DRCGraspCylinderEnvironment(cylinderOrigins, cylinderYawAngles_degrees, cylinderPitchAngles_degrees, cylinderRollAngles_degrees);
   }

   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspXAxisCylinder() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final GraspCylinderBehavior graspBehavior = new GraspCylinderBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), drcBehaviorTestHelper.getYoTime());

      RobotSide robotSideOfGraspingHand = RobotSide.LEFT;
      
      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableStaticCylinderRobot cylinder = (ContactableStaticCylinderRobot) testEnvironment.getEnvironmentRobots().get(0);
      RigidBodyTransform cylinderTransformToWorld = new RigidBodyTransform();
      cylinder.getBodyTransformToWorld(cylinderTransformToWorld);  
      TransformReferenceFrame cylinderFrame = new TransformReferenceFrame("cylinder", ReferenceFrame.getWorldFrame(), cylinderTransformToWorld);
         
      
      FramePoint graspPoint = new FramePoint(cylinderFrame, 0.0, 0.0, 0.0);
      FrameVector cylinderLongAxis = new FrameVector(cylinderFrame, 0.0, 0.0, 1.0);

      graspBehavior.initialize();
      graspBehavior.setGraspPose(robotSideOfGraspingHand, graspPoint, cylinderLongAxis, false);

      assertTrue(graspBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspBehavior);
      assertTrue(success);

      assertTrue(graspBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspYAxisCylinder() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final GraspCylinderBehavior graspBehavior = new GraspCylinderBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), drcBehaviorTestHelper.getYoTime());

      RobotSide robotSideOfGraspingHand = RobotSide.LEFT;
      
      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableStaticCylinderRobot cylinder = (ContactableStaticCylinderRobot) testEnvironment.getEnvironmentRobots().get(1);
      RigidBodyTransform cylinderTransformToWorld = new RigidBodyTransform();
      cylinder.getBodyTransformToWorld(cylinderTransformToWorld);     
      TransformReferenceFrame cylinderFrame = new TransformReferenceFrame("cylinder", ReferenceFrame.getWorldFrame(), cylinderTransformToWorld);
         
      
      FramePoint graspPoint = new FramePoint(cylinderFrame, 0.0, 0.0, 0.0);
      FrameVector cylinderLongAxis = new FrameVector(cylinderFrame, 0.0, 0.0, 1.0);

      graspBehavior.initialize();
      graspBehavior.setGraspPose(robotSideOfGraspingHand, graspPoint, cylinderLongAxis, false);

      assertTrue(graspBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspBehavior);
      assertTrue(success);

      assertTrue(graspBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testGraspZAxisCylinder() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      final GraspCylinderBehavior graspBehavior = new GraspCylinderBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            drcBehaviorTestHelper.getSDFFullRobotModel(), drcBehaviorTestHelper.getYoTime());

      RobotSide robotSideOfGraspingHand = RobotSide.LEFT;
      
      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableStaticCylinderRobot cylinder = (ContactableStaticCylinderRobot) testEnvironment.getEnvironmentRobots().get(2);
      RigidBodyTransform cylinderTransformToWorld = new RigidBodyTransform();
      cylinder.getBodyTransformToWorld(cylinderTransformToWorld);     
      TransformReferenceFrame cylinderFrame = new TransformReferenceFrame("cylinder", ReferenceFrame.getWorldFrame(), cylinderTransformToWorld);
         
      
      FramePoint graspPoint = new FramePoint(cylinderFrame, 0.0, 0.0, 0.0);
      FrameVector cylinderLongAxis = new FrameVector(cylinderFrame, 0.0, 0.0, 1.0);

      graspBehavior.initialize();
      graspBehavior.setGraspPose(robotSideOfGraspingHand, graspPoint, cylinderLongAxis, false);

      assertTrue(graspBehavior.hasInputBeenSet());

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(graspBehavior);
      assertTrue(success);

      assertTrue(graspBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }
}
