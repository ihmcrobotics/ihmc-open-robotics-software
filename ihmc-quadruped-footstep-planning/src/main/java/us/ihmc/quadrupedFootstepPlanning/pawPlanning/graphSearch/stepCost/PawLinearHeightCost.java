package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawLinearHeightCost implements PawNodeCost
{
   private final PawStepPlannerParametersReadOnly parameters;
   private final PawNodeSnapperReadOnly snapper;

   public PawLinearHeightCost(PawStepPlannerParametersReadOnly parameters, PawNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public double compute(PawNode startNode, PawNode endNode)
   {
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant();

      int endNodeXIndex = endNode.getXIndex(movingQuadrant);
      int endNodeYIndex = endNode.getYIndex(movingQuadrant);
      int startNodeXIndex = startNode.getXIndex(movingQuadrant);
      int startNodeYIndex = startNode.getYIndex(movingQuadrant);

      PawNodeSnapData endNodeData = snapper.getSnapData(endNodeXIndex, endNodeYIndex);
      PawNodeSnapData startNodeData = snapper.getSnapData(startNodeXIndex, startNodeYIndex);

      if (startNodeData == null || endNodeData == null)
         return 0.0;

      Point3D snappedStartNode = new Point3D(startNode.getX(movingQuadrant), startNode.getY(movingQuadrant), 0.0);
      Point3D snappedEndNode = new Point3D(endNode.getX(movingQuadrant), endNode.getY(movingQuadrant), 0.0);

      startNodeData.getSnapTransform().transform(snappedStartNode);
      endNodeData.getSnapTransform().transform(snappedEndNode);

      double heightChange = snappedEndNode.getZ() - snappedStartNode.getZ();

      if (heightChange > 0.0)
         return parameters.getStepUpWeight() * heightChange;
      else
         return -parameters.getStepDownWeight() * heightChange;
   }
}
