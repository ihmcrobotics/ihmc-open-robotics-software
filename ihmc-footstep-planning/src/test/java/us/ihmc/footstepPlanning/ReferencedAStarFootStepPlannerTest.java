package us.ihmc.footstepPlanning;

import org.apache.xpath.operations.Mod;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ReferenceBasedIdealStepCalculator;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ReferencedAStarFootStepPlannerTest
{
   public static final double EPS_XY = 0.5 * LatticePoint.gridSizeXY;
   public static final double EPS_YAW = 0.5 * LatticePoint.gridSizeYaw;
   public static FootstepPlan NOMINAL_PLAN = null;
   public static final Pose3D leftNominalGoalPose = new Pose3D(0.6, 0.1, 0.0, 0.0, 0.0, 0.0);
   public static final Pose3D rightNominalGoalPose = new Pose3D(0.6, -0.1, 0.0, 0.0, 0.0, 0.0);
   public static FootstepPlannerRequest REQUEST;
   public static FootstepPlanningModule MODULE;
   public static boolean TESTED_NOMINAL = false;

   @Test
   @Order(1)
   public void testNominalAStarPlanner()
   {
      REQUEST = new FootstepPlannerRequest();

      REQUEST.setStartFootPose(RobotSide.LEFT, new Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
      REQUEST.setStartFootPose(RobotSide.RIGHT, new Pose3D(0.0, -0.1, 0.0, 0.0, 0.0, 0.0));

      REQUEST.setGoalFootPose(RobotSide.LEFT, leftNominalGoalPose);
      REQUEST.setGoalFootPose(RobotSide.RIGHT, rightNominalGoalPose);

      REQUEST.setRequestedInitialStanceSide(RobotSide.LEFT);
      REQUEST.setReferencePlan(null);

      MODULE = new FootstepPlanningModule("testerModule");
      ReferenceBasedIdealStepCalculator calculator = MODULE.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator();
      calculator.setUseReferencePlan(false);
      FootstepPlannerOutput outputA = MODULE.handleRequest(REQUEST);
      int numSteps = outputA.getFootstepPlan().getNumberOfSteps();

      PlannedFootstep leftFinalStep = outputA.getFootstepPlan().getFootstep(numSteps - 2);
      PlannedFootstep rightFinalStep = outputA.getFootstepPlan().getFootstep(numSteps - 1);
      assertTrue(leftNominalGoalPose.epsilonEquals(leftFinalStep.getFootstepPose(), EPS_XY) &&
                 rightNominalGoalPose.epsilonEquals(rightFinalStep.getFootstepPose(), EPS_XY));

      NOMINAL_PLAN = outputA.getFootstepPlan();
      TESTED_NOMINAL = true;
   }

   @Test
   @Order(2)
   public void testStepGenerationFromReference()
   {
      Random random = new Random(54352);
      int index = random.nextInt() % NOMINAL_PLAN.getNumberOfSteps();

      FootstepPlan perturbedPlan = perturbPlan(NOMINAL_PLAN, index);
      if (perturbedPlan != null)
      {
         REQUEST.setReferencePlan(perturbedPlan);
         ReferenceBasedIdealStepCalculator calculator = MODULE.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator();
         calculator.setUseReferencePlan(true);
         for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
         {
            calculator.setReferenceAlpha(alpha);
            FramePose3D outputStepPose = MODULE.handleRequest(REQUEST).getFootstepPlan().getFootstep(index).getFootstepPose();
            FramePose3D nominalStepPose = NOMINAL_PLAN.getFootstep(index).getFootstepPose();
            FramePose3D perturbedStepPose = perturbedPlan.getFootstep(index).getFootstepPose();
            RobotSide side = perturbedPlan.getFootstep(index).getRobotSide();

            Point3D expectedTranslation = new Point3D();
            expectedTranslation.setX(perturbedStepPose.getX() * alpha + nominalStepPose.getX() * (1 - alpha));
            expectedTranslation.setY(perturbedStepPose.getY() * alpha + nominalStepPose.getY() * (1 - alpha));
            double expectedYaw = AngleTools.interpolateAngle(nominalStepPose.getYaw(), perturbedStepPose.getYaw(), alpha);

            // need to snap to grid
            DiscreteFootstep expectedStep = new DiscreteFootstep(expectedTranslation.getX(), expectedTranslation.getY(), expectedYaw, side);

            assertTrue("pose does not match expected at alpha: " + alpha,
                       EuclidCoreTools.epsilonEquals(outputStepPose.getTranslationX(), expectedStep.getX(), EPS_XY) &&
                                 EuclidCoreTools.epsilonEquals(outputStepPose.getTranslationY(), expectedStep.getY(), EPS_XY) &&
                                 EuclidCoreTools.epsilonEquals(outputStepPose.getYaw(), expectedStep.getYaw(), EPS_YAW));
         }
      }

   }


   private FootstepPlan perturbPlan(FootstepPlan plan, int index)
   {


      // perturb one random step from the given plan. This step will be tested against output from the planner.
      int numSteps = plan.getNumberOfSteps();
      if (index >= 0 && index < numSteps)
      {
         FootstepPlan perturbedPlan = new FootstepPlan(plan);
         Random random = new Random(123421);
         PlannedFootstep step = perturbedPlan.getFootstep(index);
         Point3D translationToAppend = EuclidCoreRandomTools.nextPoint3D(random, 0.2, 0.2, 0);
         double yawToAppend = EuclidCoreRandomTools.nextDouble(random, Math.PI / 20);
         step.getFootstepPose().appendTranslation(translationToAppend);
         step.getFootstepPose().appendYawRotation(yawToAppend);
         return perturbedPlan;
      }
      else
      {
         LogTools.warn("index of step to perturb is out of bounds !");
         return null;
      }
   }
}
