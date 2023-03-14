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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

public class ReferenceBasedIdealStepCalculator implements IdealStepCalculatorInterface
{
   private final IdealStepCalculator nominalIdealStepCalculator;
   private FootstepPlan referenceFootstepPlan;
   private DirectedGraph<FootstepGraphNode> footstepGraph;
   private DiscreteFootstep nominalIdealStep;
   private final YoBoolean stepSideIncorrect;

   // TODO add to footstep planner parameters
   /* Weight factor for using reference vs nominal ideal steps. Alpha = 0 means use nominal ideal step, alpha = 1 means use reference value */
   private double referenceAlpha;

   public ReferenceBasedIdealStepCalculator(double referenceAlpha, IdealStepCalculator nominalIdealStepCalculator, YoRegistry registry)
   {
      this.nominalIdealStepCalculator = nominalIdealStepCalculator;
      stepSideIncorrect = new YoBoolean("stepSideIncorrect", registry);
      this.referenceAlpha = referenceAlpha;
   }

   @Override
   public DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing)
   {
      nominalIdealStep = nominalIdealStepCalculator.computeIdealStep(stanceNode, startOfSwing);

      if (referenceFootstepPlan == null || referenceFootstepPlan.isEmpty() || referenceAlpha == 0.0)
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

      stepSideIncorrect.set(referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide());
      if (stepSideIncorrect.getBooleanValue())
      {
         throw new RuntimeException("Wrong side from reference plan, this should not happen ! ! !");
      }
      FramePose3D referenceFootstepPose = referenceFootstep.getFootstepPose();
      LogTools.warn("! ! USING REFERENCE STEP (alpha: {}) ! !", referenceAlpha);

      double idealStepX = EuclidCoreTools.interpolate(nominalIdealStep.getX(), referenceFootstepPose.getX(), referenceAlpha);
      double idealStepY = EuclidCoreTools.interpolate(nominalIdealStep.getY(), referenceFootstepPose.getY(), referenceAlpha);
      double idealStepYaw = AngleTools.interpolateAngle(nominalIdealStep.getYaw(), referenceFootstepPose.getYaw(), referenceAlpha);

      return new DiscreteFootstep(idealStepX, idealStepY, idealStepYaw, referenceFootstep.getRobotSide());
   }

   public void setReferenceFootstepPlan(FootstepPlan referenceFootstepPlan)
   {
      this.referenceFootstepPlan = referenceFootstepPlan;
   }

   public void clearReferencePlan()
   {
      referenceFootstepPlan = null;
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
