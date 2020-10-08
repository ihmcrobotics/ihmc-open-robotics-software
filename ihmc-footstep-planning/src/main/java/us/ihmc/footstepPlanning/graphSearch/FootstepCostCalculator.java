package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.ToDoubleFunction;
import java.util.function.UnaryOperator;

public class FootstepCostCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapperReadOnly snapper;
   private final UnaryOperator<FootstanceNode> idealStepCalculator;
   private final ToDoubleFunction<FootstanceNode> heuristics;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final RigidBodyTransform stanceNodeTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateNodeTransform = new RigidBodyTransform();
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble idealStepHeuristicCost = new YoDouble("idealStepHeuristicCost", registry);

   public FootstepCostCalculator(FootstepPlannerParametersReadOnly parameters,
                                 FootstepNodeSnapperReadOnly snapper,
                                 UnaryOperator<FootstanceNode> idealStepCalculator,
                                 ToDoubleFunction<FootstanceNode> heuristics,
                                 SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons,
                                 YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.idealStepCalculator = idealStepCalculator;
      this.heuristics = heuristics;
      this.footPolygons = footPolygons;
      parentRegistry.addChild(registry);
   }

   public double computeCost(FootstanceNode parentNode, FootstanceNode candidateNode)
   {
      FootstanceNode idealStep = idealStepCalculator.apply(parentNode);

      FootstepNodeTools.getSnappedNodeTransform(parentNode.getStanceNode(), snapper.snapFootstepNode(parentNode.getStanceNode()).getSnapTransform(), stanceNodeTransform);
      FootstepNodeTools.getSnappedNodeTransform(candidateNode.getStanceNode(), snapper.snapFootstepNode(candidateNode.getStanceNode()).getSnapTransform(), candidateNodeTransform);
      idealStepTransform.getTranslation().set(idealStep.getStanceNode().getX(), idealStep.getStanceNode().getY(), stanceNodeTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getStanceNode().getYaw());

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

      edgeCost.set(0.0);
      edgeCost.add(Math.abs(xOffset * parameters.getForwardWeight()));
      edgeCost.add(Math.abs(yOffset * parameters.getLateralWeight()));
      edgeCost.add(Math.abs(zOffset * (zOffset > 0.0 ? parameters.getStepUpWeight() : parameters.getStepDownWeight())));
      edgeCost.add(Math.abs(yawOffset * parameters.getYawWeight()));
      edgeCost.add(Math.abs(pitchOffset * parameters.getPitchWeight()));
      edgeCost.add(Math.abs(rollOffset * parameters.getRollWeight()));

      edgeCost.add(computeAreaCost(candidateNode.getStanceNode()));
      edgeCost.add(parameters.getCostPerStep());

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      idealStepHeuristicCost.set(heuristics.applyAsDouble(idealStep));
      heuristicCost.set(heuristics.applyAsDouble(candidateNode));
      double deltaHeuristics = idealStepHeuristicCost.getDoubleValue() - heuristicCost.getDoubleValue();

      if(deltaHeuristics > 0.0)
      {
         edgeCost.add(deltaHeuristics);
      }
      else
      {
         // TODO
         edgeCost.set(Math.max(0.0, edgeCost.getValue() - deltaHeuristics));
      }

      totalCost.set(edgeCost.getDoubleValue() + heuristicCost.getDoubleValue());
      return edgeCost.getValue();
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

   public void resetLoggedVariables()
   {
      edgeCost.setToNaN();
      totalCost.setToNaN();
      heuristicCost.setToNaN();
   }
}
