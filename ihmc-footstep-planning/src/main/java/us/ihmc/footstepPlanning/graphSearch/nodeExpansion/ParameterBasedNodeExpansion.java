package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;
import java.util.function.UnaryOperator;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private final List<FootstepNode> fullExpansion = new ArrayList<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final UnaryOperator<FootstepNode> idealStepSupplier;
   private final IdealStepProximityComparator idealStepProximityComparator = new IdealStepProximityComparator();
   private final HashMap<FootstepNode, PartialExpansionManager> expansionManagers = new HashMap<>();

   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();

   private boolean partialExpansionEnabled;

   public ParameterBasedNodeExpansion(FootstepPlannerParametersReadOnly parameters, UnaryOperator<FootstepNode> idealStepSupplier)
   {
      this.parameters = parameters;
      this.idealStepSupplier = idealStepSupplier;
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

            if (Math.abs(x) <= parameters.getMinXClearanceFromStance() && Math.abs(y) <= parameters.getMinYClearanceFromStance())
               continue;

            double reachFraction = EuclidCoreTools.fastSquareRoot(reachSquared) / parameters.getMaximumStepReach();
            double minYawAtFullExtension = (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMinimumStepYaw();
            double maxYawAtFullExtension = (1.0 - parameters.getStepYawReductionFactorAtMaxReach()) * parameters.getMaximumStepYaw();

            double minYaw = InterpolationTools.linearInterpolate(parameters.getMinimumStepYaw(), minYawAtFullExtension, reachFraction);
            double maxYaw = InterpolationTools.linearInterpolate(parameters.getMaximumStepYaw(), maxYawAtFullExtension, reachFraction);

            for (double yaw = minYaw; yaw <= maxYaw; yaw += LatticeNode.gridSizeYaw)
            {
               xOffsets.add(x);
               yOffsets.add(y);
               yawOffsets.add(yaw);
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

      if (idealStepSupplier != null)
      {
         idealStepProximityComparator.update(stanceNode, idealStepSupplier);
         fullExpansionToPack.sort(idealStepProximityComparator);
      }
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
         int dYaw = node1.yawIndexDistance(node2);
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
