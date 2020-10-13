package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;
import java.util.function.UnaryOperator;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private final List<FootstepNode> fullExpansion = new ArrayList<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final UnaryOperator<FootstepNode> idealStepSupplier;
   private final IdealStepProximityComparator idealStepProximityComparator = new IdealStepProximityComparator();
   private final HashMap<FootstepNode, PartialExpansionManager> expansionManagers = new HashMap<>();

   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();

   private boolean partialExpansionEnabled;

   // List of accepted values for manhattan distance between ideal step and child step
   private final TIntArrayList xyExpansionMask = new TIntArrayList();
   private final TIntArrayList yawExpansionMask = new TIntArrayList();

   public ParameterBasedNodeExpansion(FootstepPlannerParametersReadOnly parameters, UnaryOperator<FootstepNode> idealStepSupplier, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.idealStepSupplier = idealStepSupplier;
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

      for (double x = parameters.getMinimumStepLength(); x <= parameters.getMaximumStepReach(); x += LatticeNode.gridSizeXY)
      {
         for (double y = parameters.getMinimumStepWidth(); y <= parameters.getMaximumStepWidth(); y += LatticeNode.gridSizeXY)
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

            for (double yaw = minYaw; yaw <= maxYaw; yaw += LatticeNode.gridSizeYaw)
            {
               double distance = FootstepNodeTools.computeDistanceBetweenFootPolygons(new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT),
                                                                                      new FootstepNode(x, y, yaw, RobotSide.LEFT),
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
   public boolean doIterativeExpansion(FootstepNode stanceNode, List<FootstepNode> expansionToPack)
   {
      if (partialExpansionEnabled)
      {
         PartialExpansionManager partialExpansionManager = expansionManagers.computeIfAbsent(stanceNode, node ->
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
         doFullExpansion(stanceNode, expansionToPack);
         return false;
      }
   }

   @Override
   public void doFullExpansion(FootstepNode stanceNode, List<FootstepNode> fullExpansionToPack)
   {
      fullExpansionToPack.clear();
      RobotSide stepSide = stanceNode.getRobotSide().getOppositeSide();

      for (int i = 0; i < xOffsets.size(); i++)
      {
         double stepLength = xOffsets.get(i);
         double stepWidth = stepSide.negateIfRightSide(yOffsets.get(i));
         double stepYaw = stepSide.negateIfRightSide(yawOffsets.get(i));
         FootstepNode childNode = constructNodeInPreviousNodeFrame(stepLength, stepWidth, stepYaw, stanceNode);

         if (!fullExpansionToPack.contains(childNode))
            fullExpansionToPack.add(childNode);
      }

      if (idealStepSupplier != null && parameters.getEnabledExpansionMask())
      {
         applyMask(fullExpansionToPack, stanceNode);
      }

      if (idealStepSupplier != null)
      {
         idealStepProximityComparator.update(stanceNode, idealStepSupplier);
         fullExpansionToPack.sort(idealStepProximityComparator);
      }
   }

   private void applyMask(List<FootstepNode> listToFilter, FootstepNode stanceNode)
   {
      FootstepNode idealStep = idealStepSupplier.apply(stanceNode);

      int minXYManhattanDistance = computeMinXYManhattanDistance(listToFilter, idealStep);
      int minYawDistance = computeMinYawDistance(listToFilter, idealStep);

      listToFilter.removeIf(node ->
                            {
                               int xyManhattanDistance = idealStep.computeXYManhattanDistance(node) - minXYManhattanDistance;
                               if (!xyExpansionMask.contains(xyManhattanDistance))
                               {
                                  return true;
                               }

                               int yawDistance = idealStep.computeYawIndexDistance(node) - minYawDistance;
                               return !yawExpansionMask.contains(yawDistance);
                            });
   }

   private static int computeMinYawDistance(List<FootstepNode> listToFilter, FootstepNode idealStep)
   {
      int minYawDistance = Integer.MAX_VALUE;
      for (int i = 0; i < listToFilter.size(); i++)
      {
         int yawDistance = idealStep.computeYawIndexDistance(listToFilter.get(i));
         if (yawDistance < minYawDistance)
            minYawDistance = yawDistance;
      }
      return minYawDistance;
   }

   private static int computeMinXYManhattanDistance(List<FootstepNode> listToFilter, FootstepNode idealStep)
   {
      int minXYManhattanDistance = Integer.MAX_VALUE;
      for (int i = 0; i < listToFilter.size(); i++)
      {
         int xyManhattanDistance = idealStep.computeXYManhattanDistance(listToFilter.get(i));
         if (xyManhattanDistance < minXYManhattanDistance)
            minXYManhattanDistance = xyManhattanDistance;
      }
      return minXYManhattanDistance;
   }

   static class IdealStepProximityComparator implements Comparator<FootstepNode>
   {
      private FootstepNode idealStep = null;

      void update(FootstepNode stanceNode, UnaryOperator<FootstepNode> idealStepSupplier)
      {
         idealStep = idealStepSupplier.apply(stanceNode);
      }

      @Override
      public int compare(FootstepNode node1, FootstepNode node2)
      {
         Objects.requireNonNull(idealStep);

         double d1 = calculateStepProximity(node1, idealStep);
         double d2 = calculateStepProximity(node2, idealStep);
         return Double.compare(d1, d2);
      }

      static double calculateStepProximity(FootstepNode node1, FootstepNode node2)
      {
         int dX = node1.getXIndex() - node2.getXIndex();
         int dY = node1.getYIndex() - node2.getYIndex();
         int dYaw = node1.computeYawIndexDistance(node2);
         return Math.abs(dX) + Math.abs(dY) + Math.abs(dYaw);
      }
   }

   private final Vector2D footstepTranslation = new Vector2D();
   private final AxisAngle footstepRotation = new AxisAngle();

   private FootstepNode constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, FootstepNode node)
   {
      footstepTranslation.set(stepLength, stepWidth);
      footstepRotation.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      footstepRotation.transform(footstepTranslation);

      return new FootstepNode(node.getX() + footstepTranslation.getX(),
                              node.getY() + footstepTranslation.getY(),
                              stepYaw + node.getYaw(),
                              node.getRobotSide().getOppositeSide());
   }
}
