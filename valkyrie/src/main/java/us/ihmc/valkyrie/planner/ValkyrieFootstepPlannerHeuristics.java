package us.ihmc.valkyrie.planner;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.valkyrie.planner.BodyPathHelper.WaypointData;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;

public class ValkyrieFootstepPlannerHeuristics
{
   private final ValkyrieAStarFootstepPlannerParameters parameters;
   private final BodyPathHelper bodyPathHelper;
   private final ValkyriePlannerEdgeData edgeData;

   public ValkyrieFootstepPlannerHeuristics(ValkyrieAStarFootstepPlannerParameters parameters, BodyPathHelper bodyPathHelper, ValkyriePlannerEdgeData edgeData)
   {
      this.parameters = parameters;
      this.bodyPathHelper = bodyPathHelper;
      this.edgeData = edgeData;
   }

   public double compute(FootstepNode node)
   {
      WaypointData waypointData = bodyPathHelper.getWaypointFromNode(node, 0.0);
      int index = waypointData.getIndex();
      Pose2DBasics goalPose = waypointData.getNominalGoalPose();
      double pathHeading = waypointData.getPathHeading();

      double heuristicCost = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()).distance(goalPose.getPosition());

      if(heuristicCost < parameters.getFinalTurnProximity())
      {
         heuristicCost += Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), goalPose.getYaw())) * Math.PI * parameters.getIdealFootstepWidth();
      }
      else
      {
         heuristicCost += Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(node.getYaw(), pathHeading)) * Math.PI * parameters.getIdealFootstepWidth();
         heuristicCost += Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pathHeading, goalPose.getYaw())) * Math.PI * parameters.getIdealFootstepWidth();
      }

      if(bodyPathHelper.hasWaypoints())
      {
         double segmentAlpha = bodyPathHelper.getBodyPathPlanHolder().getMaxAlphaFromSegmentIndex(index);
         heuristicCost += bodyPathHelper.getBodyPathPlanHolder().computePathLength(segmentAlpha);

         int remainingWaypoints = bodyPathHelper.getNumberOfPathSegments() - (index + 1);
         heuristicCost += parameters.getWaypointCost() * remainingWaypoints;
      }

      double weightedHeuristicCost = parameters.getAstarHeuristicsWeight() * heuristicCost;
      edgeData.setHeuristicCost(weightedHeuristicCost);
      return weightedHeuristicCost;
   }
}
