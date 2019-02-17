package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
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

import java.util.ArrayList;
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
      if(startNode != null && startNode.equals(node))
         return true;

      if(!hasPlanarRegions())
         return true;

      if(parameters.getMinimumDistanceFromCliffBottoms() <= 0.0 || Double.isInfinite(parameters.getCliffHeightToAvoid()))
         return true;

      double cliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      if ((cliffHeightToAvoid <= 0.0) || (minimumDistanceFromCliffBottoms <= 0.0))
         return true;

      RigidBodyTransform footTransformToWorld = new RigidBodyTransform();
      RobotQuadrant movingQuadrant = node.getMovingQuadrant();
      FootstepNodeTools.getSnappedNodeTransformToWorld(movingQuadrant, node, snapper.getSnapData(node.getXIndex(movingQuadrant), node.getYIndex(movingQuadrant)).getSnapTransform(), footTransformToWorld);

      double maximumCliffZInSoleFrame = findHighestPointInFrame(planarRegionsList, footTransformToWorld, new Point3D(),
                                                                cliffHeightToAvoid, minimumDistanceFromCliffBottoms);

      if (maximumCliffZInSoleFrame > cliffHeightToAvoid)
      {
         rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return false;
      }

      return true;
   }

   public static double findHighestPointInFrame(PlanarRegionsList planarRegionsList, RigidBodyTransform footTransformToWorld, Point3D closestCliffPointToPack,
                                                double minimumCliffHeight, double minimumDistanceFromCliffBottoms)
     {
        double maxZInSoleFrame = Double.NEGATIVE_INFINITY;
        double closestCliffPointDistance = minimumDistanceFromCliffBottoms;

        Point3D footPointInWorldFrame = new Point3D();
        footTransformToWorld.transform(footPointInWorldFrame);

        List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(footPointInWorldFrame),
                                                                                                               minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());


        planarRegionsList.findPlanarRegionsWithinEpsilonOfPoint(footPointInWorldFrame, minimumDistanceFromCliffBottoms, intersectingRegionsToPack);

        for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
        {
           Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footPointInWorldFrame, intersectingRegion);

           double heightOfPointFromFoot = closestPointInWorld.getZ() - footPointInWorldFrame.getZ();
           double distanceToPoint = footPointInWorldFrame.distanceXY(closestPointInWorld);

           if (distanceToPoint < minimumDistanceFromCliffBottoms)
           {
              if (heightOfPointFromFoot > maxZInSoleFrame)
                 maxZInSoleFrame = heightOfPointFromFoot;

              if (heightOfPointFromFoot > minimumCliffHeight && distanceToPoint < closestCliffPointDistance)
              {
                 closestCliffPointDistance = distanceToPoint;
                 closestCliffPointToPack.set(closestPointInWorld);
              }
           }
        }

        if (maxZInSoleFrame > 0.1)
           return maxZInSoleFrame;

        return maxZInSoleFrame;
     }

}
