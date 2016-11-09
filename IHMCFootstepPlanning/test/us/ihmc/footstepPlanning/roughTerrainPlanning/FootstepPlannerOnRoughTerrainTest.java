package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class FootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      // define start and goal conditions
      FramePose initialStanceFootPose = new FramePose(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(1.0 + boxSize/2.0, 0.0, stepHeight);

      // run the test
      ArrayList<FramePose> footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, initialStanceSide.getOppositeSide(), goalPose);
      assertTrue(PlanningTestTools.isGoalWithinFeet(goalPose, footstepPlan));
   }
}
