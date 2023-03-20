package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.text.DecimalFormat;
import java.util.List;

public class ReferenceBasedIdealStepCalculator implements IdealStepCalculatorInterface
{
   private final IdealStepCalculator nominalIdealStepCalculator;
   private FootstepPlan referenceFootstepPlan;
   private DirectedGraph<FootstepGraphNode> footstepGraph;
   private DiscreteFootstep nominalIdealStep;
   private final YoBoolean stepSideIncorrect;
   public static String statusMessage = "Step calculate mode: Nominal";

   // This will be used to query what reference alpha to use.
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private static final double DEFAULT_ALPHA = 0.7;

   public ReferenceBasedIdealStepCalculator(FootstepPlannerParametersBasics footstepPlannerParameters, IdealStepCalculator nominalIdealStepCalculator, YoRegistry registry)
   {
      this.nominalIdealStepCalculator = nominalIdealStepCalculator;
      stepSideIncorrect = new YoBoolean("stepSideIncorrect", registry);
      this.footstepPlannerParameters = footstepPlannerParameters;
   }

   @Override
   public DiscreteFootstep computeIdealStep(DiscreteFootstep stanceNode, DiscreteFootstep startOfSwing)
   {
      nominalIdealStep = nominalIdealStepCalculator.computeIdealStep(stanceNode, startOfSwing);

      double referenceAlpha = footstepPlannerParameters.getReferencePlanAlpha();
      if (referenceFootstepPlan == null || referenceFootstepPlan.isEmpty() || referenceAlpha == 0.0)
      {
         statusMessage = "Step calculate mode: Nominal";
         return nominalIdealStep;
      }

      // TODO check that this indexing is correct. The sides should match if setup correctly
      List<FootstepGraphNode> pathFromStart = footstepGraph.getPathFromStart(new FootstepGraphNode(startOfSwing, stanceNode));

      int stepIndexInPlan = pathFromStart.size() - 1;

      if (stepIndexInPlan >= referenceFootstepPlan.getNumberOfSteps())
      {
         statusMessage = "Step calculate mode: Nominal";
         return nominalIdealStep;
      }

      PlannedFootstep referenceFootstep = referenceFootstepPlan.getFootstep(stepIndexInPlan);

      // TODO: Indexing should match so that referencedStep's side is equal to stance Node's opposite side.
      // In case this is not true, we will try to find the closest step from either previous or next step in the reference plan.
      stepSideIncorrect.set(referenceFootstep.getRobotSide() != stanceNode.getRobotSide().getOppositeSide());
      if (stepSideIncorrect.getBooleanValue())
      {
         LogTools.error("Wrong side from reference plan, this should not happen ! ! !");
         return nominalIdealStep;
      }

      // For any reason if alpha is not a number, set it to default value.
      if (Double.isNaN(referenceAlpha))
         referenceAlpha = DEFAULT_ALPHA;
      // Ensure alpha is within the 0 ~ 1.0 range
      else if (referenceAlpha < 0)
         referenceAlpha = 0;
      else if (referenceAlpha > 1.0)
         referenceAlpha = 1.0;

      statusMessage = "Step calculate mode: Reference (alpha : " + referenceAlpha + " )";
      FramePose3D referenceFootstepPose = referenceFootstep.getFootstepPose();

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

   public DiscreteFootstep getNominalIdealStep()
   {
      return nominalIdealStep;
   }
}
