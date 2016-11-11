package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
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
      goalPose.setPosition(2.0, 0.0, 0.5);

      // run the test
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, stairCase, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(stairCase, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   public void testSimpleStepOnBox()
   {
      // create planar regions
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize/2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, -0.001);
      generator.addRectangle(5.0, 5.0); // floor plane

      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(1.0 + boxSize/2.0, 0.0, stepHeight);

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }

   public void testRandomEnvironment()
   {
      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(-4.0, 0.0, 0.0);
      goalPose.setPosition(4.0, 0.0, 0.0);

      // run the test
      PlanarRegionsList planarRegionsList = generateRandomTerrain(random);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
   }

   private PlanarRegionsList generateRandomTerrain(Random random)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, -0.001);
      generator.addRectangle(10.0, 10.0); // floor plane

      double length = RandomTools.generateRandomDouble(random, 0.3, 1.0);
      double width = RandomTools.generateRandomDouble(random, 0.3, 1.0);
      double height = RandomTools.generateRandomDouble(random, 0.3, 1.0);

      for (int i = 0; i < 100; i++)
      {
         generator.identity();

         Vector3d translationVector = RandomTools.generateRandomVector(random, -5.0, -1.0, -0.5, 5.0, 1.0, 0.0);
         generator.translate(translationVector);

         Quat4d rotation = RandomTools.generateRandomQuaternion(random, Math.toRadians(15.0));
         generator.rotate(rotation);

         generator.addCubeReferencedAtBottomMiddle(length, width, height);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }
}
