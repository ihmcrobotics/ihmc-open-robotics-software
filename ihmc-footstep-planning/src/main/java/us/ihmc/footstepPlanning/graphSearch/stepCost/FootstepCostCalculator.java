package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.IdealStepCalculatorInterface;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.ToDoubleFunction;

public class FootstepCostCalculator implements FootstepCostCalculatorInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;
   private final IdealStepCalculatorInterface idealStepCalculator;
   private final ToDoubleFunction<FootstepGraphNode> heuristics;

   private final RigidBodyTransform stanceStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();
   private final RigidBodyTransform candidateStepTransform = new RigidBodyTransform();
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble idealStepHeuristicCost = new YoDouble("idealStepHeuristicCost", registry);

   private final YoDouble xOffset = new YoDouble("xOffset", registry);
   private final YoDouble yOffset = new YoDouble("yOffset", registry);
   private final YoDouble zOffset = new YoDouble("zOffset", registry);
   private final YoDouble yawOffset = new YoDouble("yawOffset", registry);
   private final YoDouble pitchOffset = new YoDouble("pitchOffset", registry);
   private final YoDouble rollOffset = new YoDouble("rollOffset", registry);

   public FootstepCostCalculator(FootstepPlannerParametersReadOnly parameters,
                                 FootstepSnapperReadOnly snapper,
                                 IdealStepCalculatorInterface idealStepCalculator,
                                 ToDoubleFunction<FootstepGraphNode> heuristics,
                                 SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons,
                                 YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.idealStepCalculator = idealStepCalculator;
      this.heuristics = heuristics;

      parentRegistry.addChild(registry);
   }

   @Override
   public double computeCost(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      DiscreteFootstep idealStep = idealStepCalculator.computeIdealStep(stanceStep, startOfSwing);

      DiscreteFootstepTools.getSnappedStepTransform(stanceStep, snapper.snapFootstep(stanceStep).getSnapTransform(), stanceStepTransform);
      FootstepSnapDataReadOnly candidateSnapData = snapper.snapFootstep(candidateStep);
      DiscreteFootstepTools.getSnappedStepTransform(candidateStep, candidateSnapData.getSnapTransform(), candidateStepTransform);
      idealStepTransform.getTranslation().set(idealStep.getX(), idealStep.getY(), stanceStepTransform.getTranslationZ());
      idealStepTransform.getRotation().setToYawOrientation(idealStep.getYaw());

      // calculate offset from ideal in a z-up frame
      stanceStepTransform.getRotation().setToYawOrientation(stanceStepTransform.getRotation().getYaw());
      idealStepTransform.preMultiplyInvertOther(stanceStepTransform);
      candidateStepTransform.preMultiplyInvertOther(stanceStepTransform);

      xOffset.set(candidateStepTransform.getTranslationX() - idealStepTransform.getTranslationX());
      yOffset.set(candidateStepTransform.getTranslationY() - idealStepTransform.getTranslationY());
      zOffset.set(candidateStepTransform.getTranslationZ() - idealStepTransform.getTranslationZ());
      yawOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getYaw(), idealStepTransform.getRotation().getYaw()));
      pitchOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getPitch(), idealStepTransform.getRotation().getPitch()));
      rollOffset.set(AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getRoll(), idealStepTransform.getRotation().getRoll()));

      edgeCost.set(0.0);
      edgeCost.add(Math.abs(xOffset.getValue() * parameters.getForwardWeight()));
      edgeCost.add(Math.abs(yOffset.getValue() * parameters.getLateralWeight()));
      edgeCost.add(Math.abs(zOffset.getValue() * (zOffset.getValue() > 0.0 ? parameters.getStepUpWeight() : parameters.getStepDownWeight())));
      edgeCost.add(Math.abs(yawOffset.getValue() * parameters.getYawWeight()));
      edgeCost.add(Math.abs(pitchOffset.getValue() * parameters.getPitchWeight()));
      edgeCost.add(Math.abs(rollOffset.getValue() * parameters.getRollWeight()));

      double rmsError = candidateSnapData.getSnapRMSError();
      if (!Double.isNaN(rmsError))
      {
         double rmsAlpha = EuclidCoreTools.clamp(
               (rmsError - parameters.getRMSMinErrorToPenalize()) / (parameters.getRMSErrorThreshold() - parameters.getRMSMinErrorToPenalize()), 0.0, 1.0);
         edgeCost.add(rmsAlpha * parameters.getRMSErrorCost());
      }

      edgeCost.add(computeAreaCost(candidateStep));
      edgeCost.add(parameters.getCostPerStep());

      // subtract off heuristic cost difference - i.e. ignore difference in goal proximity due to step adjustment
      idealStepHeuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, idealStep)));
      heuristicCost.set(heuristics.applyAsDouble(new FootstepGraphNode(stanceStep, candidateStep)));
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

   private double computeAreaCost(DiscreteFootstep footstep)
   {
      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(footstep);
      if (snapData != null)
      {
         double areaFraction = snapData.getSnapAreaFraction();
         if (Double.isNaN(areaFraction))
         {
            return 0.0;
         }
         double percentAreaUnoccupied = Math.max(0.0, 1.0 - areaFraction);
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
