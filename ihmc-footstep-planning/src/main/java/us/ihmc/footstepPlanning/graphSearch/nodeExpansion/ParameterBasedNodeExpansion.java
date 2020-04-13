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

public class ParameterBasedNodeExpansion
{
   private final List<FootstepNode> expansion = new ArrayList<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final IdealStepCalculator idealStepCalculator;
   private final IdealStepProximityComparator idealStepProximityComparator = new IdealStepProximityComparator();

   private final TDoubleArrayList xOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yOffsets = new TDoubleArrayList();
   private final TDoubleArrayList yawOffsets = new TDoubleArrayList();

   public ParameterBasedNodeExpansion(FootstepPlannerParametersReadOnly parameters, IdealStepCalculator idealStepCalculator)
   {
      this.parameters = parameters;
      this.idealStepCalculator = idealStepCalculator;
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
   }

   public List<FootstepNode> expandNode(FootstepNode stanceNode)
   {
      expansion.clear();
      RobotSide stepSide = stanceNode.getRobotSide().getOppositeSide();

      for (int i = 0; i < xOffsets.size(); i++)
      {
         double stepLength = xOffsets.get(i);
         double stepWidth = stepSide.negateIfRightSide(yOffsets.get(i));
         double stepYaw = stepSide.negateIfRightSide(yawOffsets.get(i));
         FootstepNode childNode = constructNodeInPreviousNodeFrame(stepLength, stepWidth, stepYaw, stanceNode);

         if (!expansion.contains(childNode))
            expansion.add(childNode);
      }

      if (idealStepCalculator != null)
      {
         idealStepProximityComparator.update(stanceNode);
         expansion.sort(idealStepProximityComparator);

         if (parameters.getMaximumBranchFactor() > 0)
         {
            while (expansion.size() > parameters.getMaximumBranchFactor())
            {
               expansion.remove(expansion.size() - 1);
            }
         }
      }

      return expansion;
   }

   private class IdealStepProximityComparator implements Comparator<FootstepNode>
   {
      private FootstepNode idealStep = null;

      void update(FootstepNode stanceNode)
      {
         idealStep = idealStepCalculator.computeIdealStep(stanceNode);
      }

      @Override
      public int compare(FootstepNode node1, FootstepNode node2)
      {
         Objects.requireNonNull(idealStep);

         int dX1 = node1.getXIndex() - idealStep.getXIndex();
         int dX2 = node2.getXIndex() - idealStep.getXIndex();

         int dY1 = node1.getYIndex() - idealStep.getYIndex();
         int dY2 = node2.getYIndex() - idealStep.getYIndex();

         int dYaw1 = node1.yawIndexDistance(idealStep);
         int dYaw2 = node2.yawIndexDistance(idealStep);

         double d1 = Math.abs(dX1) + Math.abs(dY1) + Math.abs(dYaw1);
         double d2 = Math.abs(dX2) + Math.abs(dY2) + Math.abs(dYaw2);

         return Double.compare(d1, d2);
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
