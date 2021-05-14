package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;

public class ParameterBasedStepExpansion implements FootstepExpansion
{
   private final List<FootstepGraphNode> fullExpansion = new ArrayList<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final IdealStepCalculatorInterface idealStepCalculator;
   private final IdealStepProximityComparator idealStepProximityComparator = new IdealStepProximityComparator();
   private final HashMap<FootstepGraphNode, PartialExpansionManager> expansionManagers = new HashMap<>();

   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();

   private boolean partialExpansionEnabled;

   // List of accepted values for manhattan distance between ideal step and child step
   private final TIntArrayList xyExpansionMask = new TIntArrayList();
   private final TIntArrayList yawExpansionMask = new TIntArrayList();

   public ParameterBasedStepExpansion(FootstepPlannerParametersReadOnly parameters, IdealStepCalculatorInterface idealStepCalculator, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.idealStepCalculator = idealStepCalculator;
      this.footPolygons = footPolygons;

      fillExpansionMask();
   }

   private void fillExpansionMask()
   {
      xyExpansionMask.add(0);
      xyExpansionMask.add(1);
      xyExpansionMask.add(3);
      xyExpansionMask.add(6);
      xyExpansionMask.add(13);

      yawExpansionMask.add(0);
      yawExpansionMask.add(1);
      yawExpansionMask.add(3);
   }

   public void initialize()
   {
      xOffsets.clear();
      yOffsets.clear();
      yawOffsets.clear();

      double maxReachSquared = MathTools.square(parameters.getMaximumStepReach());

      for (double x = parameters.getMinimumStepLength(); x <= parameters.getMaximumStepReach(); x += LatticePoint.gridSizeXY)
      {
         for (double y = parameters.getMinimumStepWidth(); y <= parameters.getMaximumStepWidth(); y += LatticePoint.gridSizeXY)
         {
            double relativeYToIdeal = y - parameters.getIdealFootstepWidth();
            double reachSquared = EuclidCoreTools.normSquared(x, relativeYToIdeal);
            if (reachSquared > maxReachSquared)
               continue;

            double reachFraction = EuclidCoreTools.fastSquareRoot(reachSquared) / parameters.getMaximumStepReach();
            double minYawAtFullExtension = (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMinimumStepYaw();
            double maxYawAtFullExtension = (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMaximumStepYaw();

            double minYaw = InterpolationTools.linearInterpolate(parameters.getMinimumStepYaw(), minYawAtFullExtension, reachFraction);
            double maxYaw = InterpolationTools.linearInterpolate(parameters.getMaximumStepYaw(), maxYawAtFullExtension, reachFraction);

            for (double yaw = minYaw; yaw <= maxYaw; yaw += LatticePoint.gridSizeYaw)
            {
               double distance = DiscreteFootstepTools.computeDistanceBetweenFootPolygons(new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT),
                                                                                          new DiscreteFootstep(x, y, yaw, RobotSide.LEFT),
                                                                                          footPolygons);

               if (distance >= parameters.getMinClearanceFromStance())
               {
                  xOffsets.add(x);
                  yOffsets.add(y);
                  yawOffsets.add(yaw);
               }
            }
         }
      }

      partialExpansionEnabled = parameters.getMaximumBranchFactor() > 0;
      expansionManagers.clear();
   }

   @Override
   public boolean doIterativeExpansion(FootstepGraphNode stanceStep, List<FootstepGraphNode> expansionToPack)
   {
      if (partialExpansionEnabled)
      {
         PartialExpansionManager partialExpansionManager = expansionManagers.computeIfAbsent(stanceStep, node ->
         {
            PartialExpansionManager manager = new PartialExpansionManager(parameters);
            doFullExpansion(node, fullExpansion);
            manager.initialize(fullExpansion);
            return manager;
         });
         partialExpansionManager.packPartialExpansion(expansionToPack);
         return !partialExpansionManager.finishedExpansion();
      }
      else
      {
         doFullExpansion(stanceStep, expansionToPack);
         return false;
      }
   }

   @Override
   public void doFullExpansion(FootstepGraphNode nodeToExpand, List<FootstepGraphNode> fullExpansionToPack)
   {
      fullExpansionToPack.clear();
      RobotSide stepSide = nodeToExpand.getFirstStepSide();

      for (int i = 0; i < xOffsets.size(); i++)
      {
         double stepLength = xOffsets.get(i);
         double stepWidth = stepSide.negateIfRightSide(yOffsets.get(i));
         double stepYaw = stepSide.negateIfRightSide(yawOffsets.get(i));
         DiscreteFootstep childStep = constructNodeInPreviousNodeFrame(stepLength, stepWidth, stepYaw, nodeToExpand.getSecondStep());
         FootstepGraphNode childNode = new FootstepGraphNode(nodeToExpand.getSecondStep(), childStep);

         if (!fullExpansionToPack.contains(childNode))
            fullExpansionToPack.add(childNode);
      }

      if (idealStepCalculator != null && parameters.getEnabledExpansionMask())
      {
         applyMask(fullExpansionToPack, nodeToExpand);
      }

      if (idealStepCalculator != null)
      {
         idealStepProximityComparator.update(nodeToExpand, idealStepCalculator);
         fullExpansionToPack.sort(idealStepProximityComparator);
      }
   }

   private void applyMask(List<FootstepGraphNode> listToFilter, FootstepGraphNode stanceNode)
   {
      DiscreteFootstep idealStep = idealStepCalculator.computeIdealStep(stanceNode.getSecondStep(), stanceNode.getFirstStep());

      int minXYManhattanDistance = computeMinXYManhattanDistance(listToFilter, idealStep);
      int minYawDistance = computeMinYawDistance(listToFilter, idealStep);

      listToFilter.removeIf(node ->
                            {
                               int xyManhattanDistance = idealStep.computeXYManhattanDistance(node.getSecondStep()) - minXYManhattanDistance;
                               if (!xyExpansionMask.contains(xyManhattanDistance))
                               {
                                  return true;
                               }

                               int yawDistance = idealStep.computeYawIndexDistance(node.getSecondStep()) - minYawDistance;
                               return !yawExpansionMask.contains(yawDistance);
                            });
   }

   private static int computeMinYawDistance(List<FootstepGraphNode> listToFilter, DiscreteFootstep idealStep)
   {
      int minYawDistance = Integer.MAX_VALUE;
      for (int i = 0; i < listToFilter.size(); i++)
      {
         int yawDistance = idealStep.computeYawIndexDistance(listToFilter.get(i).getSecondStep());
         if (yawDistance < minYawDistance)
            minYawDistance = yawDistance;
      }
      return minYawDistance;
   }

   private static int computeMinXYManhattanDistance(List<FootstepGraphNode> listToFilter, DiscreteFootstep idealStep)
   {
      int minXYManhattanDistance = Integer.MAX_VALUE;
      for (int i = 0; i < listToFilter.size(); i++)
      {
         int xyManhattanDistance = idealStep.computeXYManhattanDistance(listToFilter.get(i).getSecondStep());
         if (xyManhattanDistance < minXYManhattanDistance)
            minXYManhattanDistance = xyManhattanDistance;
      }
      return minXYManhattanDistance;
   }

   static class IdealStepProximityComparator implements Comparator<FootstepGraphNode>
   {
      private DiscreteFootstep idealStep = null;

      void update(FootstepGraphNode stanceNode, IdealStepCalculatorInterface idealStepCalculator)
      {
         idealStep = idealStepCalculator.computeIdealStep(stanceNode.getSecondStep(), stanceNode.getFirstStep());
      }

      @Override
      public int compare(FootstepGraphNode node1, FootstepGraphNode node2)
      {
         Objects.requireNonNull(idealStep);

         double d1 = calculateStepProximity(node1.getSecondStep(), idealStep);
         double d2 = calculateStepProximity(node2.getSecondStep(), idealStep);
         return Double.compare(d1, d2);
      }

      static double calculateStepProximity(DiscreteFootstep step1, DiscreteFootstep step2)
      {
         int dX = step1.getXIndex() - step2.getXIndex();
         int dY = step1.getYIndex() - step2.getYIndex();
         int dYaw = step1.computeYawIndexDistance(step2);
         return Math.abs(dX) + Math.abs(dY) + Math.abs(dYaw);
      }
   }

   private final Vector2D footstepTranslation = new Vector2D();
   private final AxisAngle footstepRotation = new AxisAngle();

   private DiscreteFootstep constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, DiscreteFootstep step)
   {
      footstepTranslation.set(stepLength, stepWidth);
      footstepRotation.setYawPitchRoll(step.getYaw(), 0.0, 0.0);
      footstepRotation.transform(footstepTranslation);

      return new DiscreteFootstep(step.getX() + footstepTranslation.getX(),
                              step.getY() + footstepTranslation.getY(),
                              stepYaw + step.getYaw(),
                                  step.getRobotSide().getOppositeSide());
   }
}
