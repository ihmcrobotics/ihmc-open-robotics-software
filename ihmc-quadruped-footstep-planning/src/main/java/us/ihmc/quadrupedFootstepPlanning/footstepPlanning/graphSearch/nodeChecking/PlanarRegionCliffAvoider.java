package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public class PlanarRegionCliffAvoider extends FootstepNodeChecker
{
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapperReadOnly snapper;

   private FootstepNode startNode;

   public PlanarRegionCliffAvoider(FootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransform)
   {
      this.startNode = startNode;
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      if (startNode != null && startNode.equals(node))
         return true;

      if (!hasPlanarRegions())
         return true;

      double cliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      if (!Double.isFinite(cliffHeightToAvoid))
         return true;

      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();
      double minimumDistanceFromCliffTops = parameters.getMinimumDistanceFromCliffTops();

      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      int xIndex = node.getXIndex(movingQuadrant);
      int yIndex = node.getYIndex(movingQuadrant);
      RigidBodyTransform footTransformToWorld = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransformToWorld(xIndex, yIndex, snapper.getSnapData(xIndex, yIndex).getSnapTransform(), footTransformToWorld);

      Point3D footInWorld = new Point3D();
      footTransformToWorld.transform(footInWorld);

      if (minimumDistanceFromCliffBottoms > 0.0)
      {
         Point3D highestNearbyPoint = new Point3D();
         double maximumCliffZInSoleFrame = findHighestNearbyPoint(planarRegionsList, footInWorld, highestNearbyPoint, minimumDistanceFromCliffBottoms);

         if (maximumCliffZInSoleFrame > cliffHeightToAvoid)
         {
            rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
            return false;
         }
      }

      /*
      if (minimumDistanceFromCliffTops > 0.0)
      {
         Point3D lowestNearbyPoint = new Point3D();
         double minimumCliffZInSoleFrame = findLowestNearbyPoint(planarRegionsList, footInWorld, lowestNearbyPoint, minimumDistanceFromCliffTops);

         if (minimumCliffZInSoleFrame < -cliffHeightToAvoid)
         {
            rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.AT_CLIFF_TOP);
            return false;
         }
      }
      */

      return true;
   }

   private static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, Point3DBasics highestNearbyPointToPack,
                                                double minimumDistanceFromCliffBottoms)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(footInWorld), minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();
         double distanceToPoint = footInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffBottoms && heightOfPointFromFoot > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromFoot;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }

   private static double findLowestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, Point3DBasics lowestNearbyPointToPack,
                                               double minimumDistanceFromCliffTops)
   {
      double minZInSoleFrame = Double.POSITIVE_INFINITY;

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(footInWorld), minimumDistanceFromCliffTops, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();
         double distanceToPoint = footInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffTops && heightOfPointFromFoot < minZInSoleFrame)
         {
            minZInSoleFrame = heightOfPointFromFoot;
            lowestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return minZInSoleFrame;
   }
}
