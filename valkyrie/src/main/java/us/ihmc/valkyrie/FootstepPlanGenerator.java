package us.ihmc.valkyrie;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlanGenerator
{
   public FootstepPlanGenerator()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule("planningModule");
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setStartFootPose(RobotSide.RIGHT, new Pose3D(0.0, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.LEFT, new Pose3D(0.6, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.RIGHT, new Pose3D(0.6, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);

      FootstepPlan referencePlan = new FootstepPlan();

      // Reference plan with alpha 1
//      referencePlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, -0.05, 0.0, 0.0, 0.0, 0.0)));
//      referencePlan.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, 0.25, 0.0, 0.0, 0.0, 0.0)));
//      request.setReferencePlan(referencePlan);

      // Reference plan with alpha 0.5
      referencePlan.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.3, -0.2, 0.0, 0.0, 0.0, 0.0)));
      referencePlan.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.6, 0.2, 0.0, 0.0, 0.0, 0.0)));
      request.setReferencePlan(referencePlan);

      // Plan without a reference:
      // RIGHT ( 0.300, -0.100,  0.000 )
      // LEFT  ( 0.600,  0.100,  0.000 )
      // RIGHT ( 0.600, -0.100,  0.000 )

      FootstepPlannerOutput output = planningModule.handleRequest(request);
      logger.logSession();

//      printOutput(output);
   }

   private void printOutput(FootstepPlannerOutput output)
   {
      FootstepPlan outputPlan = output.getFootstepPlan();
      for (int i = 0; i < outputPlan.getNumberOfSteps(); i++)
      {
         PlannedFootstep step = outputPlan.getFootstep(i);
         System.out.println(step.getRobotSide() + ", \n" + step.getFootstepPose());
      }
   }

   public static void main(String[] args)
   {
      new FootstepPlanGenerator();
   }
}
