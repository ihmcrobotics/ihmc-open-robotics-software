package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashSet;
import java.util.function.DoubleSupplier;

public class ParameterBasedNodeExpansion implements FootstepNodeExpansion
{
   private final HashSet<FootstepNode> expansion = new HashSet<>();

   private final DoubleSupplier minimumStepLength;
   private final DoubleSupplier maximumStepReach;
   private final DoubleSupplier stepYawReductionFactorAtMaxReach;
   private final DoubleSupplier minimumStepYaw;
   private final DoubleSupplier maximumStepYaw;
   private final DoubleSupplier minimumStepWidth;
   private final DoubleSupplier maximumStepWidth;
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier minXClearanceFromStance;
   private final DoubleSupplier minYClearanceFromStance;

   public ParameterBasedNodeExpansion(FootstepPlannerParametersReadOnly parameters)
   {
      this(parameters::getMinimumStepLength,
           parameters::getMaximumStepReach,
           parameters::getStepYawReductionFactorAtMaxReach,
           parameters::getMinimumStepYaw,
           parameters::getMaximumStepYaw,
           parameters::getMinimumStepWidth,
           parameters::getMaximumStepWidth,
           parameters::getIdealFootstepWidth,
           parameters::getMinXClearanceFromStance,
           parameters::getMinYClearanceFromStance);
   }

   public ParameterBasedNodeExpansion(DoubleSupplier minimumStepLength,
                                      DoubleSupplier maximumStepReach,
                                      DoubleSupplier stepYawReductionFactorAtMaxReach,
                                      DoubleSupplier minimumStepYaw,
                                      DoubleSupplier maximumStepYaw,
                                      DoubleSupplier minimumStepWidth,
                                      DoubleSupplier maximumStepWidth,
                                      DoubleSupplier idealFootstepWidth,
                                      DoubleSupplier minXClearanceFromStance,
                                      DoubleSupplier minYClearanceFromStance)
   {
      this.minimumStepLength = minimumStepLength;
      this.maximumStepReach = maximumStepReach;
      this.stepYawReductionFactorAtMaxReach = stepYawReductionFactorAtMaxReach;
      this.minimumStepYaw = minimumStepYaw;
      this.maximumStepYaw = maximumStepYaw;
      this.minimumStepWidth = minimumStepWidth;
      this.maximumStepWidth = maximumStepWidth;
      this.idealFootstepWidth = idealFootstepWidth;
      this.minXClearanceFromStance = minXClearanceFromStance;
      this.minYClearanceFromStance = minYClearanceFromStance;
   }

   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      expansion.clear();

      RobotSide nextSide = node.getRobotSide().getOppositeSide();
      double maxReachSquared = MathTools.square(maximumStepReach.getAsDouble());
      double minYawAtFullExtension = (1.0 - stepYawReductionFactorAtMaxReach.getAsDouble()) * minimumStepYaw.getAsDouble();
      double maxYawAtFullExtension = (1.0 - stepYawReductionFactorAtMaxReach.getAsDouble()) * maximumStepYaw.getAsDouble();
      for (double x = minimumStepLength.getAsDouble(); x <= maximumStepReach.getAsDouble(); x += LatticeNode.gridSizeXY)
      {
         for (double y = minimumStepWidth.getAsDouble(); y <= maximumStepWidth.getAsDouble(); y += LatticeNode.gridSizeXY)
         {
            double relativeYToIdeal = y - idealFootstepWidth.getAsDouble();
            double reachSquared = EuclidCoreTools.normSquared(x, relativeYToIdeal);
            if (reachSquared > maxReachSquared)
               continue;

            if (Math.abs(x) <= minXClearanceFromStance.getAsDouble() && Math.abs(y) <= minYClearanceFromStance.getAsDouble())
               continue;

            double reachFraction = EuclidCoreTools.fastSquareRoot(reachSquared) / maximumStepReach.getAsDouble();
            double minYaw = InterpolationTools.linearInterpolate(minimumStepYaw.getAsDouble(), minYawAtFullExtension, reachFraction);
            double maxYaw = InterpolationTools.linearInterpolate(maximumStepYaw.getAsDouble(), maxYawAtFullExtension, reachFraction);

            for (double yaw = minYaw; yaw <= maxYaw; yaw += LatticeNode.gridSizeYaw)
            {
               FootstepNode offsetNode = constructNodeInPreviousNodeFrame(x, nextSide.negateIfRightSide(y), nextSide.negateIfRightSide(yaw), node);
               expansion.add(offsetNode);
            }
         }
      }

      return expansion;
   }

   private static FootstepNode constructNodeInPreviousNodeFrame(double stepLength, double stepWidth, double stepYaw, FootstepNode node)
   {
      Vector2D footstep = new Vector2D(stepLength, stepWidth);
      AxisAngle rotation = new AxisAngle(node.getYaw(), 0.0, 0.0);
      rotation.transform(footstep);

      return new FootstepNode(node.getX() + footstep.getX(), node.getY() + footstep.getY(), stepYaw + node.getYaw(), node.getRobotSide().getOppositeSide());
   }
}
