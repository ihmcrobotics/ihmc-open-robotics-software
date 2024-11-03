package us.ihmc.footstepPlanning.simplePlanners;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;

public class TurnWalkTurnPlannerTest
{
   @Test
   public void testWalking45Degrees()
   {
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      TurnWalkTurnPlanner planner = new TurnWalkTurnPlanner(parameters);

      FramePose3D startPoseRight = new FramePose3D();
      FramePose3D startPoseLeft = new FramePose3D();
      startPoseRight.setY(-parameters.getIdealFootstepWidth() / 2.0);
      startPoseLeft.setY(parameters.getIdealFootstepWidth() / 2.0);

      {
         planner.setInitialStanceFoot(startPoseRight, RobotSide.RIGHT);

         FramePose3D goalPose = new FramePose3D();
         goalPose.getPosition().set(1.5, 1.5, 0.0);

         planner.setGoal(goalPose);

         planner.plan();

         FootstepPlan plan = planner.getPlan();

         // first step is the left side
         RobotSide expectedStepSide = RobotSide.LEFT;
         int stepIdx = 0;
         // we know the heading is 45 degrees. We're turning to that position now
         while (true)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());

            // make sure we've turned CCW
            assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() > 0.0);
            // make sure each step is turning more CCW
            if (stepIdx > 0)
            {
               assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() > plan.getFootstep(stepIdx - 1).getFootstepPose().getYaw());
            }

            // once we've achieved the desired angle, stop turning
            if (plan.getFootstep(stepIdx).getFootstepPose().getYaw() > Math.toRadians(45.0) - 1e-5)
               break;

            expectedStepSide = expectedStepSide.getOppositeSide();
            stepIdx++;
         }

         // do the steps walking straight
         int forwardSteps = (int) Math.ceil(goalPose.getPosition().distanceFromOrigin() / parameters.getIdealFootstepLength());
         for (; stepIdx <= forwardSteps + 1; stepIdx++)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());
            assertEquals(plan.getFootstep(stepIdx).getFootstepPose().getYaw(), Math.toRadians(45.0), 1e-5);
            expectedStepSide = expectedStepSide.getOppositeSide();
         }

         // we know the heading is 45 degrees. We're turning back from that position now
         while (true)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());

            // make sure we've turned CCW
            assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() < Math.toRadians(45.0));
            // make sure each step is turning more CCW
            if (stepIdx > 0)
            {
               assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() < plan.getFootstep(stepIdx - 1).getFootstepPose().getYaw());
            }

            // once we've achieved the zero angle, stop turning
            if (plan.getFootstep(stepIdx).getFootstepPose().getYaw() < 1e-5)
               break;

            stepIdx++;
            expectedStepSide = expectedStepSide.getOppositeSide();
         }
      }

      {
         planner.setInitialStanceFoot(startPoseLeft, RobotSide.LEFT);

         FramePose3D goalPose = new FramePose3D();
         goalPose.getPosition().set(1.5, 1.5, 0.0);

         planner.setGoal(goalPose);

         planner.plan();

         FootstepPlan plan = planner.getPlan();

         // first step is the left side
         RobotSide expectedStepSide = RobotSide.RIGHT;
         int stepIdx = 0;
         // we know the heading is 45 degrees. We're turning to that position now
         while (true)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());

            // make sure we've turned CCW
            assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() > 0.0);
            // make sure each step is turning more CCW
            if (stepIdx > 0)
            {
               assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() > plan.getFootstep(stepIdx - 1).getFootstepPose().getYaw());
            }

            // once we've achieved the desired angle, stop turning
            if (plan.getFootstep(stepIdx).getFootstepPose().getYaw() > Math.toRadians(45.0) - 1e-5)
               break;

            expectedStepSide = expectedStepSide.getOppositeSide();
            stepIdx++;
         }

         // do the steps walking straight
         int forwardSteps = (int) Math.ceil(goalPose.getPosition().distanceFromOrigin() / parameters.getIdealFootstepLength());
         for (; stepIdx <= forwardSteps + 1; stepIdx++)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());
            assertEquals(plan.getFootstep(stepIdx).getFootstepPose().getYaw(), Math.toRadians(45.0), 1e-5);
            expectedStepSide = expectedStepSide.getOppositeSide();
         }

         // we know the heading is 45 degrees. We're turning back from that position now
         while (true)
         {
            assertEquals(expectedStepSide, plan.getFootstep(stepIdx).getRobotSide());

            // make sure we've turned CCW
            assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() < Math.toRadians(45.0));
            // make sure each step is turning more CCW
            if (stepIdx > 0)
            {
               assertTrue(plan.getFootstep(stepIdx).getFootstepPose().getYaw() < plan.getFootstep(stepIdx - 1).getFootstepPose().getYaw());
            }

            // once we've achieved the zero angle, stop turning
            if (plan.getFootstep(stepIdx).getFootstepPose().getYaw() < 1e-5)
               break;

            stepIdx++;
            expectedStepSide = expectedStepSide.getOppositeSide();
         }
      }
   }

}
