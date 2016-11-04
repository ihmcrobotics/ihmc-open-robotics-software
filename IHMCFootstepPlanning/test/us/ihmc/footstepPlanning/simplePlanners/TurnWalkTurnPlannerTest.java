package us.ihmc.footstepPlanning.simplePlanners;

import org.junit.Test;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
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


   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testIsGoalWithinFeet()
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
      FootstepPlanner turnWalkTurnPlanner = new TurnWalkTurnPlanner();
      turnWalkTurnPlanner.setGoalPose(goalPose);
      turnWalkTurnPlanner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);

      List<FramePose2d> footstepPlan = turnWalkTurnPlanner.plan();

      FramePose2d lastFoostep = footstepPlan.get(footstepPlan.size()-1);
      FramePose2d secondLastFoostep = footstepPlan.get(footstepPlan.size()-2);

      FramePose2d achievedGoal = new FramePose2d();

      double distanceBetweenFootsteps = lastFoostep.getPositionDistance(secondLastFoostep);
      double distanceFromLastStepToGoal = lastFoostep.getPositionDistance(goalPose);
      achievedGoal.interpolate(lastFoostep, secondLastFoostep, 0.5);

      System.out.println("achievedgoal: " + achievedGoal.getX() + " " + achievedGoal.getY() + " " + achievedGoal.getYaw() + " " + achievedGoal.getReferenceFrame() );
      System.out.println("goalpose: " + goalPose.getX() + " " + goalPose.getY()+ " " + goalPose.getYaw() + " " + goalPose.getReferenceFrame());
      System.out.println("lastfootstep: " + lastFoostep.getX() + " " + lastFoostep.getY()+ " " + lastFoostep.getYaw() + " " + lastFoostep.getReferenceFrame());
      System.out.println("secondlastfootstep: " + secondLastFoostep.getX() + " " + secondLastFoostep.getY()+ " " + secondLastFoostep.getYaw() + " " + secondLastFoostep.getReferenceFrame());

      new FlatFootstepPlanVisualizer(goalPose, initialStanceFootPose, initialStanceFootSide);
      assertTrue("Achieved pose is not equal to goal pose", achievedGoal.epsilonEquals(goalPose, 10E-2));
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
