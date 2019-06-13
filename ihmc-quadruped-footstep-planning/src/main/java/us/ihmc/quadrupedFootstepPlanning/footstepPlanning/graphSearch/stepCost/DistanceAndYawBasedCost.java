package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class DistanceAndYawBasedCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public DistanceAndYawBasedCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();

      Point2DReadOnly startXGaitCenter = startNode.getOrComputeXGaitCenterPoint();

      Vector2D offsetToFoot = new Vector2D(movingQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()), movingQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      offsetToFoot.scale(0.5);

      startNode.getStepOrientation().transform(offsetToFoot);

      Point2D nominalStartFoot = new Point2D(startXGaitCenter);
      nominalStartFoot.add(offsetToFoot);

      double stepX = nominalStartFoot.getX() - endNode.getX(movingQuadrant);
      double stepY = nominalStartFoot.getY() - endNode.getY(movingQuadrant);

      double stepDistance = EuclidCoreTools.norm(stepX, stepY);
      double stepYaw = endNode.getStepYaw() - startNode.getStepYaw();

      // don't include yaw, because this is captured in the step distance, too
      return plannerParameters.getDistanceHeuristicWeight() * stepDistance;// + plannerParameters.getYawWeight() * stepYaw;
   }
}
