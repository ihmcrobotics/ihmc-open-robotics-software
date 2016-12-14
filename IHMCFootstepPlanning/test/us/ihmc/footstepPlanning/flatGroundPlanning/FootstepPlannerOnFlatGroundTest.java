package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point2d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class FootstepPlannerOnFlatGroundTest implements PlanningTest
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double stepWidth = 0.3;
   private final Random random = new Random(727434726273L);

   public void testJustStraightLine()
   {
      testJustStraightLine(true);
   }

   public void testJustStraightLine(boolean assertPlannerReturnedResult)
   {
      double xGoal = 5.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = 0.0;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);

      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testATightTurn()
   {
      testJustStraightLine(true);
   }

   public void testATightTurn(boolean assertPlannerReturnedResult)
   {
      double xGoal = 1.0;
      double yGoal = 0.5;
      double yawGoal = 0.0;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
      if (assertPlannerReturnedResult) assertTrue(footstepPlan.getNumberOfSteps() < 30);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
   }

   public void testStraightLineWithInitialTurn()
   {
      testStraightLineWithInitialTurn(true);
   }

   public void testStraightLineWithInitialTurn(boolean assertPlannerReturnedResult)
   {
      double xGoal = 5.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(20.0);
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = Math.toRadians(20.0);
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testJustTurnInPlace()
   {
      testJustTurnInPlace(true);
   }

   public void testJustTurnInPlace(boolean assertPlannerReturnedResult)
   {
      double xGoal = 0.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(160.0);
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }

   public void testRandomPoses()
   {
      testRandomPoses(true);
   }

   public void testRandomPoses(boolean assertPlannerReturnedResult)
   {
      double xGoal = random.nextDouble();
      double yGoal = random.nextDouble();
      double yawGoal = 0.0;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = random.nextDouble();
      double yInitialStanceFoot = random.nextDouble();
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.generateRandomRobotSide(random);

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose3d, initialStanceFootSide, goalPose3d, null, assertPlannerReturnedResult);

      if (visualize())
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose3d);
      if (assertPlannerReturnedResult) assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose3d, footstepPlan));
   }
}
