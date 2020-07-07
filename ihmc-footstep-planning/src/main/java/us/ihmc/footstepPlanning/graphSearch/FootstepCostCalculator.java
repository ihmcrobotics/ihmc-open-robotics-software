package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.IdealStepCalculator;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.ToDoubleFunction;
import java.util.function.UnaryOperator;

public class FootstepCostCalculator
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;
   private final UnaryOperator<FootstepNode> idealStepCalculator;
   private final ToDoubleFunction<FootstepNode> heuristics;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final FootstepPlannerEdgeData edgeData;

   private final RigidBodyTransform stanceNodeTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateNodeTransform = new RigidBodyTransform();

   public FootstepCostCalculator(FootstepPlannerParametersReadOnly parameters,
                                 FootstepNodeSnapperReadOnly snapper,
                                 UnaryOperator<FootstepNode> idealStepCalculator,
                                 ToDoubleFunction<FootstepNode> heuristics,
                                 SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons,
                                 FootstepPlannerEdgeData edgeData)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.idealStepCalculator = idealStepCalculator;
      this.heuristics = heuristics;
      this.footPolygons = footPolygons;
      this.edgeData = edgeData;
   }

   public double computeCost(FootstepNode stanceNode, FootstepNode candidateNode)
   {
      FootstepNode idealStep = idealStepCalculator.apply(stanceNode);

      FootstepNodeTools.getSnappedNodeTransform(stanceNode, snapper.snapFootstepNode(stanceNode).getSnapTransform(), stanceNodeTransform);
      FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapper.snapFootstepNode(candidateNode).getSnapTransform(), candidateNodeTransform);
      idealStepTransform.getTranslation().set(idealStep.getX(), idealStep.getY(), stanceNodeTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getYaw());

      // calculate offset from ideal in a z-up frame
      stanceNodeTransform.getRotation().setToYawOrientation(stanceNodeTransform.getRotation().getYaw());
      idealStepTransform.preMultiplyInvertOther(stanceNodeTransform);
      candidateNodeTransform.preMultiplyInvertOther(stanceNodeTransform);

      double xOffset = candidateNodeTransform.getTranslationX() - idealStepTransform.getTranslationX();
      double yOffset = candidateNodeTransform.getTranslationY() - idealStepTransform.getTranslationY();
      double zOffset = candidateNodeTransform.getTranslationZ() - idealStepTransform.getTranslationZ();
      double yawOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateNodeTransform.getRotation().getYaw(), idealStepTransform.getRotation().getYaw());
      double pitchOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateNodeTransform.getRotation().getPitch(), idealStepTransform.getRotation().getPitch());
      double rollOffset = AngleTools.computeAngleDifferenceMinusPiToPi(candidateNodeTransform.getRotation().getRoll(), idealStepTransform.getRotation().getRoll());

      double cost = 0.0;
      cost += Math.abs(xOffset * parameters.getForwardWeight());
      cost += Math.abs(yOffset * parameters.getLateralWeight());
      cost += Math.abs(zOffset * (zOffset > 0.0 ? parameters.getStepUpWeight() : parameters.getStepDownWeight()));
      cost += Math.abs(yawOffset * parameters.getYawWeight());
      cost += Math.abs(pitchOffset * parameters.getPitchWeight());
      cost += Math.abs(rollOffset * parameters.getRollWeight());

      cost += computeAreaCost(candidateNode);
      cost += parameters.getCostPerStep();

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      double deltaHeuristics = heuristics.applyAsDouble(idealStep) - heuristics.applyAsDouble(candidateNode);
      if(deltaHeuristics > 0.0)
      {
         cost += deltaHeuristics;
      }
      else
      {
         cost = Math.max(0.0, cost - deltaHeuristics);
      }

      if (edgeData != null)
         edgeData.setEdgeCost(cost);

      return cost;
   }

   private double computeAreaCost(FootstepNode footstepNode)
   {
      FootstepNodeSnapDataReadOnly snapData = snapper.snapFootstepNode(footstepNode);
      if (snapData != null)
      {
         ConvexPolygon2DReadOnly footholdAfterSnap = snapData.getCroppedFoothold();
         if(footholdAfterSnap.isEmpty() || footholdAfterSnap.containsNaN())
         {
            return 0.0;
         }

         double area = footholdAfterSnap.getArea();
         double footArea = footPolygons.get(footstepNode.getRobotSide()).getArea();

         if (footholdAfterSnap.isEmpty())
            return 0.0;

         double percentAreaUnoccupied = Math.max(0.0, 1.0 - area / footArea);
         return percentAreaUnoccupied * parameters.getFootholdAreaWeight();
      }
      else
      {
         return 0.0;
      }
   }
}
