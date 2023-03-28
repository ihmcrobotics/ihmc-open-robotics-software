package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ReferenceBasedIdealStepCalculator;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.File;
import java.util.ArrayList;

/*
Use this to generate footsteps with nominal and referenced AStar for debugging / visualizing.
View the generated logs from RemoteFootstepPlannerUI
 */

public class ReferencedAStarFootstepGeneratorForDebugging
{
   public ReferencedAStarFootstepGeneratorForDebugging()
   {
      ArrayList<FootstepPlan> referenced_output_plans = new ArrayList<>();

      FootstepPlanningModule planningModule = new FootstepPlanningModule("planningModule");
      FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setStartFootPose(RobotSide.RIGHT, new Pose3D(0.0, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.LEFT, new Pose3D(0.6, 0.1, 0.0, 0.0, 0.0, 0.0));
      request.setGoalFootPose(RobotSide.RIGHT, new Pose3D(0.6, -0.1, 0.0, 0.0, 0.0, 0.0));
      request.setRequestedInitialStanceSide(RobotSide.LEFT);

      // Case: NOMINAL plan
      FootstepPlan nominal_output_plan = planAndLog(0.0, planningModule, request, logger, "nominal");

      // Case: REFERENCE plan A
      FootstepPlan reference_A = new FootstepPlan();
      reference_A.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.3, -0.2, 0.0, 0.0, 0.0, 0.0)));
      reference_A.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.6, 0.2, 0.0, 0.0, 0.0, 0.0)));
      request.setReferencePlan(reference_A);

      // ref A alpha 1
      FootstepPlan ref_A_alpha_full_output = planAndLog(1.0, planningModule, request, logger, "ref_A_alpha_full");
      referenced_output_plans.add(ref_A_alpha_full_output);
      // ref A alpha 0.5
      FootstepPlan ref_A_alpha_half_output = planAndLog(0.5, planningModule, request, logger, "ref_A_alpha_half");
      referenced_output_plans.add(ref_A_alpha_half_output);
      // ref A alpha 0.0
      FootstepPlan ref_A_alpha_zero_output = planAndLog(0.0, planningModule, request, logger, "ref_A_alpha_zero");
      referenced_output_plans.add(ref_A_alpha_zero_output);

      // Case: REFERENCE plan B
      FootstepPlan reference_B = new FootstepPlan();
      reference_B.addFootstep(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, -0.05, 0.0, 0.0, 0.0, 0.0)));
      reference_B.addFootstep(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Pose3D(0.25, 0.25, 0.0, 0.0, 0.0, 0.0)));
      request.setReferencePlan(reference_B);

      // ref B alpha 1
      FootstepPlan ref_B_alpha_full_output = planAndLog(1.0, planningModule, request, logger, "ref_B_alpha_full");
      referenced_output_plans.add(ref_B_alpha_full_output);
      // ref B alpha 0.5
      FootstepPlan ref_B_alpha_half_output = planAndLog(0.5, planningModule, request, logger, "ref_B_alpha_half");
      referenced_output_plans.add(ref_B_alpha_half_output);
      // ref B alpha 0.0
      FootstepPlan ref_B_alpha_zero_output = planAndLog(0.0, planningModule, request, logger, "ref_B_alpha_zero");
      referenced_output_plans.add(ref_B_alpha_zero_output);

      // Plan without a reference:
      // RIGHT ( 0.300, -0.100,  0.000 )
      // LEFT  ( 0.600,  0.100,  0.000 )
      // RIGHT ( 0.600, -0.100,  0.000 )

      System.out.println("NOMINAL output");
      printPlan(nominal_output_plan);
      System.out.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      System.out.println("REF outputs with alphas 1.0, 0.5 ,0.0");
      for (FootstepPlan plan : referenced_output_plans)
      {
         printPlan(plan);
         System.out.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      }
   }

   private void printPlan(FootstepPlan plan)
   {
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         PlannedFootstep step = plan.getFootstep(i);
         System.out.println(step.getRobotSide() + ", \n" + step.getFootstepPose());
      }
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

   private FootstepPlan planAndLog(double alpha, FootstepPlanningModule planningModule, FootstepPlannerRequest request, FootstepPlannerLogger logger, String folderName)
   {
      planningModule.getFootstepPlannerParameters().setReferencePlanAlpha(alpha);
      FootstepPlannerOutput output = planningModule.handleRequest(request);
      FootstepPlan plan = new FootstepPlan(output.getFootstepPlan());

      String defaultSaveDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
      String logDirectory = defaultSaveDirectory + folderName + File.separator;
      logger.logSessionWithExactFolderName(logDirectory);

      return plan;
   }

   public static void main(String[] args)
   {
      new ReferencedAStarFootstepGeneratorForDebugging();
   }
}


