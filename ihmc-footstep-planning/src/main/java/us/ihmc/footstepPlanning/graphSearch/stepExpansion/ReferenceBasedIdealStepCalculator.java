package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

public class ReferenceBasedIdealStepCalculator implements IdealStepCalculatorInterface
{
   private final IdealStepCalculator nominalIdealStepCalculator;
   private final YoBoolean stepSideIncorrect;
   // This will be used to query what reference alpha to use.
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final YoBoolean usingReferenceStep;
   private FootstepPlan referenceFootstepPlan;
   private DirectedGraph<FootstepGraphNode> footstepGraph;
   private DiscreteFootstep nominalIdealStep;

   public ReferenceBasedIdealStepCalculator(DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                                            IdealStepCalculator nominalIdealStepCalculator,
                                            YoRegistry registry)
   {
      this.nominalIdealStepCalculator = nominalIdealStepCalculator;
      stepSideIncorrect = new YoBoolean("stepSideIncorrect", registry);
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.usingReferenceStep = new YoBoolean("usingReferenceStep", registry);
   }

   @Override
   public DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing)
   {
      nominalIdealStep = nominalIdealStepCalculator.computeIdealStep(stanceNode, startOfSwing);

      double referenceAlpha = footstepPlannerParameters.getReferencePlanAlpha();
      if (referenceFootstepPlan == null || referenceFootstepPlan.isEmpty() || referenceAlpha == 0.0)
      {
         return nominalIdealStep;
      }

      PlannedFootstep referenceFootstep = getReferenceStep(new FootstepGraphNode(startOfSwing, stanceNode));
      if (referenceFootstep == null)
      {
         return nominalIdealStep;
      }

      // TODO: Indexing should match so that referencedStep's side is equal to stance Node's opposite side.
      // In case this is not true, we will try to find the closest step from either previous or next step in the reference plan.
      stepSideIncorrect.set(referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide());
      if (stepSideIncorrect.getBooleanValue())
      {
         throw new RuntimeException("Invalid side on reference plan");
      }

      double nominalIdealStepX = nominalIdealStep.getX();
      double nominalIdealStepY = nominalIdealStep.getY();
      double nominalIdealStepYaw = nominalIdealStep.getYaw();
      double interpolatedYaw = AngleTools.interpolateAngle(nominalIdealStepYaw, referenceFootstep.getFootstepPose().getYaw(), referenceAlpha);

      Pose2D nominalStepPose = new Pose2D(nominalIdealStep.getX(), nominalIdealStep.getY(), nominalIdealStep.getYaw());
      Pose2D referenceStepPose = new Pose2D(referenceFootstep.getFootstepPose());
      Pose2D interpolatedPose = new Pose2D(nominalStepPose);
      interpolatedPose.interpolate(referenceStepPose, referenceAlpha);
      return new DiscreteFootstep(interpolatedPose.getX(), interpolatedPose.getY(), interpolatedPose.getYaw(), referenceFootstep.getRobotSide());
   }

   public void setReferenceFootstepPlan(FootstepPlan referenceFootstepPlan)
   {
      this.referenceFootstepPlan = referenceFootstepPlan;
   }

   public void setFootstepGraph(DirectedGraph<FootstepGraphNode> footstepGraph)
   {
      this.footstepGraph = footstepGraph;
   }

   public boolean isUsingReferenceStep()
   {
      return usingReferenceStep.getBooleanValue();
   }

   public PlannedFootstep getReferenceStep(FootstepGraphNode graphNode)
   {
      if (referenceFootstepPlan == null)
         return null;

      List<FootstepGraphNode> pathFromStart = footstepGraph.getPathFromStart(graphNode);
      int stepIndexInPlan = pathFromStart.size() - 1;
      if (stepIndexInPlan < referenceFootstepPlan.getNumberOfSteps())
      {
         return referenceFootstepPlan.getFootstep(stepIndexInPlan);
      }
      else
      {
         return null;
      }
   }

   public DiscreteFootstep getNominalIdealStep()
   {
      return nominalIdealStep;
   }
}
