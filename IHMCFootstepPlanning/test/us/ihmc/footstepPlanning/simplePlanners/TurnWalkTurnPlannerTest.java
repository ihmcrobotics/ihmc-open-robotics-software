package us.ihmc.footstepPlanning.simplePlanners;

import org.junit.Test;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point2d;
import java.util.List;
import java.util.Random;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class TurnWalkTurnPlannerTest
{
   private final Random random = new Random(727434726273L);
   private boolean succes = true;
   private boolean showSimulation = true;

   private double stepWidth = 0.3;


   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      double xGoal = 5.0;
      double yGoal = 0.0;
      double yawGoal = 0.0;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = stepWidth/2.0;
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.RIGHT;

      assertTrue(isGoalWithinFeet(goalPose, initialStanceFootPose, initialStanceFootSide));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustTurnInPlace()
   {
      double xGoal = 0.0;
      double yGoal = 0.0;
      double yawGoal = 0.0;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = stepWidth/2.0;
      double yawInitial = 20;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.RIGHT;

      assertTrue(isGoalWithinFeet(goalPose, initialStanceFootPose, initialStanceFootSide));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testRandomPoses()
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

      assertTrue(isGoalWithinFeet(goalPose, initialStanceFootPose, initialStanceFootSide));
   }

   public boolean isGoalWithinFeet(FramePose2d goalPose, FramePose2d initialStanceFootPose, RobotSide initialStanceFootSide)
   {

      FootstepPlanner turnWalkTurnPlanner = new TurnWalkTurnPlanner();
      turnWalkTurnPlanner.setGoalPose(goalPose);
      turnWalkTurnPlanner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);

      List<FramePose2d> footstepPlan = turnWalkTurnPlanner.plan();

      FramePose2d lastFoostep = footstepPlan.get(footstepPlan.size() - 1);
      FramePose2d secondLastFoostep = footstepPlan.get(footstepPlan.size()-2);

      FramePose2d achievedGoal = new FramePose2d();

      achievedGoal.interpolate(lastFoostep, secondLastFoostep, 0.5);

      if(showSimulation)
         new FlatFootstepPlanVisualizer(goalPose, initialStanceFootPose, initialStanceFootSide);
      if(achievedGoal.epsilonEquals(goalPose, 10E-2))
         succes = true;
      else succes = false;

      return succes;
   }

   private ConvexPolygon2d makeFootPolygon()
   {
      ConvexPolygon2d ret = new ConvexPolygon2d();
      ret.addVertex(new Point2d(0.1, 0.05));
      ret.addVertex(new Point2d(-0.1, 0.05));
      ret.addVertex(new Point2d(0.1, -0.05));
      ret.addVertex(new Point2d(-0.1, -0.05));
      ret.update();
      return ret;
   }

}
