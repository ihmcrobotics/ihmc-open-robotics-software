package us.ihmc.footstepPlanning;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;

public class ReferencedAStarFootStepPlannerTest
{
   // The epsilons are multiplied by 0.5 so that we allow a max epsilon of half the distance between two grid points, we hope to round to the nearest grid point so if we are greater than 0.5 we should round up
   private static final double EPSILON = 0.5 * LatticePoint.gridSizeXY;
   private static final double EPSLILON_YAW = 0.5 * LatticePoint.gridSizeYaw;
   private static final Pose3D leftNominalGoalPose = new Pose3D(0.6, 0.1, 0.0, 0.0, 0.0, 0.0);
   private static final Pose3D rightNominalGoalPose = new Pose3D(0.6, -0.1, 0.0, 0.0, 0.0, 0.0);
   private static final FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
   private static final FootstepPlanningModule footstepPlannerModule = new FootstepPlanningModule("testerModule");

   private static FootstepPlannerOutput plannerOutput;

   @BeforeAll
   public static void generateAStarPlan()
   {
      // Here we set up parameters to test the specific case we need
      footstepPlannerRequest.setStartFootPose(RobotSide.LEFT, new Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
      footstepPlannerRequest.setStartFootPose(RobotSide.RIGHT, new Pose3D(0.0, -0.1, 0.0, 0.0, 0.0, 0.0));

      footstepPlannerRequest.setGoalFootPose(RobotSide.LEFT, leftNominalGoalPose);
      footstepPlannerRequest.setGoalFootPose(RobotSide.RIGHT, rightNominalGoalPose);

      footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.LEFT);
      footstepPlannerRequest.setReferencePlan(null);
      footstepPlannerRequest.setAssumeFlatGround(true);

      plannerOutput = footstepPlannerModule.handleRequest(footstepPlannerRequest);
   }

   /**
    * This test is here to make sure the planner is working, its nothing special and could probably be removed if this case is covered in another class
    */
   @Test
   public void testNominalAStarPlanner()
   {
      int numSteps = plannerOutput.getFootstepPlan().getNumberOfSteps();

      PlannedFootstep leftFinalStep = plannerOutput.getFootstepPlan().getFootstep(numSteps - 2);
      PlannedFootstep rightFinalStep = plannerOutput.getFootstepPlan().getFootstep(numSteps - 1);
      assertTrue(leftNominalGoalPose.epsilonEquals(leftFinalStep.getFootstepPose(), EPSILON));
      assertTrue(rightNominalGoalPose.epsilonEquals(rightFinalStep.getFootstepPose(), EPSILON));
   }

   /**
    * This test is rather complex, here we are testing that the
    * {@link us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters#referencePlanAlpha} works as expected given a translation in the
    * x, the y and a rotation about the yaw axis.
    */
   @Test
   public void testReferenceAlphaXYAndYaw()
   {
      // Here we take the nominal plan (the expected plan given no reference plan) and we save the first step in that plan to compare against later
      // The reason we take the first step is that it's not a starting pose, and its not a goal post, so it won't be affected by those kind of parameters
      FootstepPlan nominalPlan = plannerOutput.getFootstepPlan();
      int index = 0;
      FramePose3D nominalStepPose = nominalPlan.getFootstep(index).getFootstepPose();

      // We take the adjusted plan and the adjusted footstep pose
      FootstepPlan adjustedPlan = adjustFootstepAtIndex(nominalPlan, index);
      FramePose3D adjustedStepPose = adjustedPlan.getFootstep(index).getFootstepPose();

      // Now we want to set the adjusted plan as a reference, this way the planner will try to use the reference plan
      // Specifically the adjusted step that we changed, that is what we will be comparing against
      footstepPlannerRequest.setReferencePlan(adjustedPlan);

      // We want to test that the reference alpha is working as expected, and the range of its values are from 0 to 1 so we will loop through all of these values
      for (double referenceAlpha = 0.0; referenceAlpha <= 1.0; referenceAlpha += 0.1)
      {
         // Each loop through we update the reference alpha to the new value, then we plan new footsteps and get the output pose at the index we care about
         footstepPlannerModule.getFootstepPlannerParameters().setReferencePlanAlpha(referenceAlpha);
         FootstepPlannerOutput output = footstepPlannerModule.handleRequest(footstepPlannerRequest);
         FramePose3D outputStep = output.getFootstepPlan().getFootstep(index).getFootstepPose();

         // We calculate the expected adjustment using an interpolation between the nominal step and the reference step
         // This should match what the planner is doing give or take a few less parameters
         Point3D expectedTranslation = new Point3D();
         expectedTranslation.setX(EuclidCoreTools.interpolate(nominalStepPose.getX(), adjustedStepPose.getX(), referenceAlpha));
         expectedTranslation.setY(EuclidCoreTools.interpolate(nominalStepPose.getY(), adjustedStepPose.getY(), referenceAlpha));
         double expectedYaw = AngleTools.interpolateAngle(nominalStepPose.getYaw(), adjustedStepPose.getYaw(), referenceAlpha);

         assertTrue(EuclidCoreTools.epsilonEquals(outputStep.getTranslationX(), expectedTranslation.getX(), EPSILON),
                    "The actual pose was: " + outputStep.getTranslationX() + " and the expected is: " + expectedTranslation.getX() + "! referenceAlpha: "
                    + referenceAlpha);
         assertTrue(EuclidCoreTools.epsilonEquals(outputStep.getTranslationY(), expectedTranslation.getY(), EPSILON),
                    "The actual pose was: " + outputStep.getTranslationY() + " and the expected is: " + expectedTranslation.getY() + "! referenceAlpha: "
                    + referenceAlpha);
         assertTrue(EuclidCoreTools.epsilonEquals(outputStep.getYaw(), expectedYaw, EPSLILON_YAW),
                    "The actual yaw was: " + outputStep.getYaw() + " and the expected is: " + expectedYaw + "! referenceAlpha: " + referenceAlpha);
      }
   }

   /**
    * This method adjusts a footstep at the given index, this then returns a plan that will be the original plan passed in but that one index will have been
    * modified.
    *
    * @param plan  is the footstep plan that we want to modify, only the footstep at the index will be modified
    * @param index the spot at which we want to modify the step
    * @return the new footstep plan that contains the modified step
    */
   private FootstepPlan adjustFootstepAtIndex(FootstepPlan plan, int index)
   {
      // Here we check that the index is within the legal bounds of the footstep plan, don't really need to do this since its a test but whatever
      int numSteps = plan.getNumberOfSteps();
      if (index >= 0 && index < numSteps)
      {
         FootstepPlan adjustedPlan = new FootstepPlan(plan);
         PlannedFootstep step = adjustedPlan.getFootstep(index);

         // We append a translation to the foot that we are trying to modify
         Point3D translationToAppend = new Point3D(LatticePoint.gridSizeXY * 2, -LatticePoint.gridSizeXY * 2, 0);
         step.getFootstepPose().appendTranslation(translationToAppend);

         // This is negative because of the specific index we are using, this is hacky and won't work for each step, because depending on the foot the yaw limits are different
         double yawToAppend = -LatticePoint.gridSizeYaw * 2;
         step.getFootstepPose().appendYawRotation(yawToAppend);

         return adjustedPlan;
      }
      else
      {
         // Again, don't really need this in the test case
         LogTools.warn("index of step to perturb is out of bounds !");
         return null;
      }
   }
}
