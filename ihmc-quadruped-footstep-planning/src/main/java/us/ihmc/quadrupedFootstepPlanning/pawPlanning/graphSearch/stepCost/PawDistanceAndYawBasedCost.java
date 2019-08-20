package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawDistanceAndYawBasedCost implements PawNodeCost
{
   private final PawPlannerParameters plannerParameters;
   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public PawDistanceAndYawBasedCost(PawPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();

      Point2DReadOnly startXGaitCenter = startNode.getOrComputeXGaitCenterPoint();

      Vector2D offsetToPaw = new Vector2D(movingQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength()), movingQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth()));
      offsetToPaw.scale(0.5);

      startNode.getStepOrientation().transform(offsetToPaw);

      Point2D nominalStartPaw = new Point2D(startXGaitCenter);
      nominalStartPaw.add(offsetToPaw);

      double stepX = nominalStartPaw.getX() - endNode.getX(movingQuadrant);
      double stepY = nominalStartPaw.getY() - endNode.getY(movingQuadrant);

      double stepDistance = EuclidCoreTools.norm(stepX, stepY);
      double stepYaw = endNode.getStepYaw() - startNode.getStepYaw();

      // don't include yaw, because this is captured in the step distance, too
      return plannerParameters.getDistanceWeight() * stepDistance;// + plannerParameters.getYawWeight() * stepYaw;
   }
}
