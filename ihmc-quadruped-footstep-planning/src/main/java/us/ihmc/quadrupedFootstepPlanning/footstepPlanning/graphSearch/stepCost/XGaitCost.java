package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import com.vividsolutions.jts.geomgraph.Quadrant;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class XGaitCost implements FootstepCost
{
   private final FootstepPlannerParameters plannerParameters;
   private final FootstepNodeSnapper snapper;
   final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final QuadrantDependentList<Point3D> startFootPositions = new QuadrantDependentList<>();


   public XGaitCost(FootstepPlannerParameters plannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeSnapper snapper)
   {
      this.plannerParameters = plannerParameters;
      this.xGaitSettings = xGaitSettings;
      this.snapper = snapper;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         startFootPositions.put(robotQuadrant, new Point3D());
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

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FootstepNodeSnapData snapData = snapper.snapFootstepNode(startNode.getXIndex(robotQuadrant), startNode.getYIndex(robotQuadrant));
         startFootPositions.get(robotQuadrant).set(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);
         snapData.getSnapTransform().transform(startFootPositions.get(robotQuadrant));
      }

      double nominalPitch = QuadrupedSupportPolygon.getNominalPitch(startFootPositions, 4);
      double durationBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousQuadrant, xGaitSettings);
      double desiredSpeed = plannerParameters.getMaxWalkingSpeedMultiplier() * xGaitSettings.getMaxSpeed();

      Vector3D desiredDistance = new Vector3D(durationBetweenSteps * desiredSpeed, 0.0, 0.0);

      AxisAngle bodyOrientation = new AxisAngle(startNode.getNominalYaw(), nominalPitch, 0.0);

      bodyOrientation.transform(desiredDistance);

      Vector3D forward = new Vector3D(0.5 * (movingQuadrant.isQuadrantInFront() ? xGaitSettings.getStanceLength() : -xGaitSettings.getStanceLength()), 0.0, 0.0);
      Vector3D side = new Vector3D(0.0, 0.5 * (movingQuadrant.isQuadrantOnLeftSide() ? xGaitSettings.getStanceWidth() : -xGaitSettings.getStanceWidth()), 0.0);

      bodyOrientation.transform(forward);
      bodyOrientation.transform(side);

      Point3D endXGaitCenter = new Point3D(startXGaitCenter);
      endXGaitCenter.add(desiredDistance);

      Point3D endFoot = new Point3D(endXGaitCenter);
      endFoot.add(forward);
      endFoot.add(side);

      return plannerParameters.getXGaitWeight() * (MathTools.square(endFoot.getX() - endNode.getX(movingQuadrant)) + MathTools.square(endFoot.getY() - endNode.getY(movingQuadrant)));
   }
}
