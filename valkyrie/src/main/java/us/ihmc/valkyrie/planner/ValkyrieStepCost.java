package us.ihmc.valkyrie.planner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;

import java.util.function.UnaryOperator;

public class ValkyrieStepCost implements FootstepCost
{
   private final ValkyrieAStarFootstepPlannerParameters parameters;
   private final FootstepNodeSnapperReadOnly snapper;
   private final UnaryOperator<FootstepNode> idealStepCalculator;
   private final ValkyrieFootstepPlannerHeuristics heuristics;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final RigidBodyTransform stanceNodeTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateNodeTransform = new RigidBodyTransform();

   private ValkyriePlannerEdgeData edgeData;

   public ValkyrieStepCost(ValkyrieAStarFootstepPlannerParameters parameters,
                           FootstepNodeSnapperReadOnly snapper,
                           ValkyrieFootstepPlannerHeuristics heuristics,
                           UnaryOperator<FootstepNode> idealStepCalculator,
                           SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons,
                           ValkyriePlannerEdgeData edgeData)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.heuristics = heuristics;
      this.idealStepCalculator = idealStepCalculator;
      this.footPolygons = footPolygons;
      this.edgeData = edgeData;
   }

   @Override
   public double compute(FootstepNode stanceNode, FootstepNode candidateNode)
   {
      FootstepNode idealStep = idealStepCalculator.apply(stanceNode);

      FootstepNodeTools.getSnappedNodeTransform(stanceNode, snapper.getSnapData(stanceNode).getSnapTransform(), stanceNodeTransform);
      FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapper.getSnapData(candidateNode).getSnapTransform(), candidateNodeTransform);
      idealStepTransform.setTranslation(idealStep.getX(), idealStep.getY(), stanceNodeTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getYaw());

      // calculate offset from ideal in a z-up frame
      stanceNodeTransform.getRotation().setToYawOrientation(stanceNodeTransform.getRotation().getYaw());
      idealStepTransform.preMultiplyInvertOther(stanceNodeTransform);
      candidateNodeTransform.preMultiplyInvertOther(stanceNodeTransform);

      double xOffset = candidateNodeTransform.getTranslationX() - idealStepTransform.getTranslationX();
      double yOffset = candidateNodeTransform.getTranslationY() - idealStepTransform.getTranslationY();
      double zOffset = candidateNodeTransform.getTranslationZ() - idealStepTransform.getTranslationZ();
      double yawOffset = candidateNodeTransform.getRotation().getYaw() - idealStepTransform.getRotation().getYaw();
      double pitchOffset = candidateNodeTransform.getRotation().getPitch() - idealStepTransform.getRotation().getPitch();
      double rollOffset = candidateNodeTransform.getRotation().getRoll() - idealStepTransform.getRotation().getRoll();

      double cost = 0.0;
      cost += Math.abs(parameters.getTranslationWeight().getX() * xOffset);
      cost += Math.abs(parameters.getTranslationWeight().getY() * yOffset);
      cost += Math.abs(parameters.getTranslationWeight().getZ() * zOffset);
      cost += Math.abs(parameters.getOrientationWeight().getYaw() * yawOffset);
      cost += Math.abs(parameters.getOrientationWeight().getPitch() * pitchOffset);
      cost += Math.abs(parameters.getOrientationWeight().getRoll() * rollOffset);

      cost += computeAreaCost(candidateNode);
      cost += parameters.getCostPerStep();

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      double deltaHeuristics = heuristics.compute(idealStep) - heuristics.compute(candidateNode);
      if(deltaHeuristics > 0.0)
         cost += deltaHeuristics;
      else
         cost = Math.max(0.0, cost - deltaHeuristics);

      edgeData.setEdgeCost(cost);
      return cost;
   }

   private double computeAreaCost(FootstepNode footstepNode)
   {
      FootstepNodeSnapData snapData = snapper.getSnapData(footstepNode);
      if (snapData != null)
      {
         ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
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
