package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class FootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   private static final Random random = new Random(42747621889239430L);
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public void testOnStaircase()
   {
      testOnStaircase(new Vector3d(), true);
   }

   public void testOnStaircase(Vector3d rotationVector, boolean assertPlannerReturnedResult)
   {
      PlanarRegionsList stairCase = PlanarRegionsListExamples.generateStairCase(rotationVector);

      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(2.0, -0.2, 0.53);

      // run the test
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, stairCase, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(stairCase, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   public void testOverCinderBlockField(boolean assertPlannerReturnedResult)
   {
      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;
      PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks);

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(9.0, 0.0, 0.0);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   public void testSimpleStepOnBox()
   {
      testSimpleStepOnBox(true);
   }

   public void testSimpleStepOnBox(boolean assertPlannerReturnedResult)
   {
      // create planar regions
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane

      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(1.0 + boxSize / 2.0, 0.0, stepHeight);

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }


   public void testSimpleStepOnBoxTwo(boolean assertPlannerReturnedResult)
   {
      // create planar regions
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.PI/4.0, Axis.Z);
      generator.translate(-6.0 + 1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane

      // define start and goal conditions
      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.rotate(Math.PI/4.0, Axis.Z);
      RigidBodyTransform transform = transformGenerator.getRigidBodyTransformCopy();
      Point3d startPosition = new Point3d(-6.0, 0.0, 0.0);
      transform.transform(startPosition);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(startPosition);
      initialStanceFootPose.setOrientation(new AxisAngle4d(new Vector3d(0.0, 0.0, 1.0), Math.PI/4.0));

      Point3d goalPosition = new Point3d(-6.0 + 1.0 + boxSize / 2.0, 0.0, stepHeight);
      transform.transform(goalPosition);

      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(goalPosition);
      goalPose.setOrientation(new AxisAngle4d(new Vector3d(0.0, 0.0, 1.0), Math.PI/4.0));

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   public void testRandomEnvironment()
   {
      testRandomEnvironment(true);
   }

   public void testRandomEnvironment(boolean assertPlannerReturnedResult)
   {
      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(-6.0, 0.0, 0.0);
      goalPose.setPosition(6.0, 0.0, 0.0);

      // run the test
      PlanarRegionsList planarRegionsList = generateRandomTerrain(random);
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
   }

   public void testSimpleGaps()
   {
      testSimpleGaps(true);
   }

   public void testSimpleGaps(boolean assertPlannerReturnedResult)
   {
      // create planar regions
      double boxHeight = 0.2;
      double boxSize = 0.87;
      double gapSize = 0.2;
      int numberOfGaps = 6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, boxHeight);

      for (int i=0; i<numberOfGaps; i++)
      {
         generator.translate(boxSize + gapSize, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, boxHeight);
      }

//      generator.identity();
//      generator.translate(0.0, 0.0, 0.001);
//      generator.addRectangle(5.0, 5.0); // floor plane

      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, 0.0, boxHeight);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(numberOfGaps * (boxSize + gapSize), 0.0, boxHeight);
      goalPose.setOrientation(new AxisAngle4d(new Vector3d(0.0, 0.0, 1.0), Math.PI));

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   private PlanarRegionsList generateRandomTerrain(Random random)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(14.0, 3.0); // floor plane

      double length = RandomTools.generateRandomDouble(random, 0.3, 1.0);
      double width = RandomTools.generateRandomDouble(random, 0.3, 1.0);
      double height = RandomTools.generateRandomDouble(random, 0.07, 0.3);

      for (int i = 0; i < 100; i++)
      {
         generator.identity();

         Vector3d translationVector = RandomTools.generateRandomVector(random, -5.0, -1.0, -0.05, 5.0, 1.0, 0.0);
         generator.translate(translationVector);

         Quat4d rotation = RandomTools.generateRandomQuaternion(random, Math.toRadians(15.0));
         generator.rotate(rotation);

         generator.addCubeReferencedAtBottomMiddle(length, width, height);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }
}
