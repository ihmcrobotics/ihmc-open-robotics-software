package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   final QuadrupedXGaitSettingsReadOnly xGaitSettings;

   public XGaitCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();
      RobotQuadrant previousQuadrant = startNode.getMovingQuadrant();
      if (movingQuadrant.getNextReversedRegularGaitSwingQuadrant() != previousQuadrant)
      { // fixme this should account for different phases.
         throw new RuntimeException("For some reason the feet movement is out of order.");
      }

      Point2DReadOnly startXGaitCenter = startNode.getOrComputeXGaitCenterPoint();

      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousQuadrant, xGaitSettings);
      double desiredSpeed = plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();

      Vector2D desiredDistance = new Vector2D(durationBetweenSteps * desiredSpeed, 0.0);

      AxisAngle bodyOrientation = new AxisAngle(startNode.getNominalYaw(), 0.0, 0.0);

      bodyOrientation.transform(desiredDistance);

      Vector2D forward = new Vector2D(0.5 * (movingQuadrant.isQuadrantInFront() ? xGaitSettings.getStanceLength() : -xGaitSettings.getStanceLength()), 0.0);
      Vector2D side = new Vector2D(0.0, 0.5 * (movingQuadrant.isQuadrantOnLeftSide() ? xGaitSettings.getStanceWidth() : -xGaitSettings.getStanceWidth()));

      bodyOrientation.transform(forward);
      bodyOrientation.transform(side);

      Point2D endXGaitCenter = new Point2D(startXGaitCenter);
      endXGaitCenter.add(desiredDistance);

      Point2D endFoot = new Point2D(endXGaitCenter);
      endFoot.add(forward);
      endFoot.add(side);

      return plannerParameters.getXGaitWeight() * (MathTools.square(endFoot.getX() - endNode.getX(movingQuadrant)) + MathTools.square(endFoot.getY() - endNode.getY(movingQuadrant)));
   }
}
