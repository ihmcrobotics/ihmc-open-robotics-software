package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
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
      FramePose3D referenceFootstepPose = referenceFootstep.getFootstepPose();

      // TODO: figure out if the indexing should match.
      // If not, search for closest step?
      if (referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide())
      {
         throw new RuntimeException("Wrong side from referencePlan");
      }

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
