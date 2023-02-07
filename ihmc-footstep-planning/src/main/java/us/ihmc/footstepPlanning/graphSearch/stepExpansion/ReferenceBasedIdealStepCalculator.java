package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.robotics.geometry.AngleTools;

import java.util.List;

public class ReferenceBasedIdealStepCalculator implements IdealStepCalculatorInterface
{
   private final IdealStepCalculator nominalIdealStepCalculator;
   private FootstepPlan referenceFootstepPlan;
   private DirectedGraph<FootstepGraphNode> footstepGraph;
   private DiscreteFootstep nominalIdealStep;

   // TODO add to footstep planner parameters
   /* Weight factor for using reference vs nominal ideal steps. Alpha = 0 means use nominal ideal step, alpha = 1 means use reference value */
   private double referenceAlpha = 0.5;

   public ReferenceBasedIdealStepCalculator(IdealStepCalculator nominalIdealStepCalculator)
   {
      this.nominalIdealStepCalculator = nominalIdealStepCalculator;
   }

   @Override
   public DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing)
   {
      nominalIdealStep = nominalIdealStepCalculator.computeIdealStep(stanceNode, startOfSwing);

      if (referenceFootstepPlan == null || referenceFootstepPlan.isEmpty())
      {
         return nominalIdealStep;
      }

      // TODO check that this indexing is correct. The sides should match if setup correctly
      List<FootstepGraphNode> pathFromStart = footstepGraph.getPathFromStart(new FootstepGraphNode(startOfSwing, stanceNode));

      int stepIndexInPlan = pathFromStart.size() - 1;

      if (stepIndexInPlan >= referenceFootstepPlan.getNumberOfSteps())
      {
         return nominalIdealStep;
      }

      PlannedFootstep referenceFootstep = referenceFootstepPlan.getFootstep(stepIndexInPlan);

      // TODO: Indexing should match so that referencedStep's side is equal to stance Node's opposite side.
      //  In case this is not true, we will try to find the closest step from either previous or next step in the reference plan.
      if (referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide())
      {
//         throw new RuntimeException("Wrong side from referencePlan");
         LogTools.warn("Wrong side from reference plan . . .\n Trying to fetch from one step previous or after this within the reference plan . . .");
         int index_a = stepIndexInPlan + 1;
         PlannedFootstep candidate_a = null;
         int index_b = stepIndexInPlan - 1;
         PlannedFootstep candidate_b = null;
         if (index_a < referenceFootstepPlan.getNumberOfSteps())
         {
            candidate_a = referenceFootstepPlan.getFootstep(index_a);
         }
         if (index_b >= 0)
         {
            candidate_b = referenceFootstepPlan.getFootstep(index_b);
         }

         // Only previous step available to fetch.
         if (candidate_a == null && candidate_b != null)
         {
            referenceFootstep = candidate_b;
         }
         // Only next step available to fetch.
         else if (candidate_a != null && candidate_b == null)
         {
            referenceFootstep = candidate_a;
         }
         else if (candidate_a != null)
         {
            double distance_a = referenceFootstep.getFootstepPose().getPositionDistance(candidate_a.getFootstepPose());
            double distance_b = referenceFootstep.getFootstepPose().getPositionDistance(candidate_b.getFootstepPose());

            if (distance_a <= distance_b)
            {
               referenceFootstep = candidate_a;
               LogTools.warn("Fetching Next step as reference . . .");
            }
            else
            {
               referenceFootstep = candidate_b;
               LogTools.warn("Fetching Previous step as reference . . .");
            }
         }
         else
         {
            LogTools.warn("No previous or next step to fetch from reference plan. Using nominalIdealStep . . .");
            return nominalIdealStep;
         }
      }
      FramePose3D referenceFootstepPose = referenceFootstep.getFootstepPose();
      LogTools.warn("~~~~~~~~~~~~~~~~~~~~ USING REFERENCE STEP IN THE REFERENCE_BASED_IDEAL_STEP_CALCULATOR ~~~~~~~~~~~~~~~~~~~~");

      double idealStepX = EuclidCoreTools.interpolate(nominalIdealStep.getX(), referenceFootstepPose.getX(), referenceAlpha);
      double idealStepY = EuclidCoreTools.interpolate(nominalIdealStep.getY(), referenceFootstepPose.getY(), referenceAlpha);
      double idealStepYaw = AngleTools.interpolateAngle(nominalIdealStep.getYaw(), referenceFootstepPose.getYaw(), referenceAlpha);

      return new DiscreteFootstep(idealStepX, idealStepY, idealStepYaw, referenceFootstep.getRobotSide());
   }

   public void setReferenceFootstepPlan(FootstepPlan referenceFootstepPlan)
   {
      this.referenceFootstepPlan = referenceFootstepPlan;
   }

   public void setFootstepGraph(DirectedGraph<FootstepGraphNode> footstepGraph)
   {
      this.footstepGraph = footstepGraph;
   }

   public void setReferenceAlpha(double referenceAlpha)
   {
      if (referenceAlpha <= 1.0 && referenceAlpha >= 0.0)
         this.referenceAlpha = referenceAlpha;
   }

   public double getReferenceAlpha()
   {
      return referenceAlpha;
   }

   public DiscreteFootstep getNominalIdealStep()
   {
      return nominalIdealStep;
   }
}
