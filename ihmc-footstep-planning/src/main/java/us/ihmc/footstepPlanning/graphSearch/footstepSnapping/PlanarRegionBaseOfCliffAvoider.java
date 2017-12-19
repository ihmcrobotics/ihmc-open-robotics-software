package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionBaseOfCliffAvoider implements FootstepNodeChecker
{
   private final FootstepPlannerParameters parameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private PlanarRegionsList planarRegionsList;
   private FootstepNode startNode;

   public PlanarRegionBaseOfCliffAvoider(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      this.startNode = startNode;
   }

   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      if(startNode != null && startNode.equals(node))
         return true;

      if(planarRegionsList == null)
         return true;

      if(parameters.getMinimumDistanceFromCliffBottoms() <= 0.0 || Double.isInfinite(parameters.getCliffHeightToAvoid()))
         return true;

      double cliffHeightToAvoid = parameters.getCliffHeightToAvoid();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      if ((cliffHeightToAvoid <= 0.0) || (minimumDistanceFromCliffBottoms <= 0.0))
         return true;

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(node, soleTransform);

      RigidBodyTransform inverseSoleTransform = new RigidBodyTransform(soleTransform);
      inverseSoleTransform.invert();

      ArrayList<LineSegment2D> lineSegmentsInSoleFrame = new ArrayList<>();
      ConvexPolygon2D footPolygon = footPolygons.get(node.getRobotSide());
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly point1 = footPolygon.getVertex(i);
         Point2DReadOnly point2 = footPolygon.getVertex((i + 1) % footPolygon.getNumberOfVertices());

         Point2D averagePoint = EuclidGeometryTools.averagePoint2Ds(Arrays.asList(point1, point2));
         Point2D extendedPoint = new Point2D(averagePoint);
         extendedPoint.scale(1.0 + (minimumDistanceFromCliffBottoms / averagePoint.distanceFromOrigin()));
         lineSegmentsInSoleFrame.add(new LineSegment2D(averagePoint.getX(), averagePoint.getY(), extendedPoint.getX(), extendedPoint.getY()));
      }

      Point3D highestPointInSoleFrame = new Point3D();
      LineSegment2D highestLineSegmentInSoleFrame = new LineSegment2D();

      double maximumCliffZInSoleFrame = findHighestPointInOriginalSoleFrame(planarRegionsList, soleTransform, inverseSoleTransform, lineSegmentsInSoleFrame, highestPointInSoleFrame, highestLineSegmentInSoleFrame);
      return maximumCliffZInSoleFrame < cliffHeightToAvoid;
   }
   
   private double findHighestPointInOriginalSoleFrame(PlanarRegionsList planarRegionsList, RigidBodyTransform soleTransform, RigidBodyTransform inverseSoleTransform, ArrayList<LineSegment2D> lineSegmentsInSoleFrame,
                                                      Point3D highestPointInSoleFrameToPack, LineSegment2D highestLineSegmentInSoleFrameToPack)
     {
        double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

        LineSegment2D lineSegmentInWorldFrame = new LineSegment2D();
        Point3D pointOneInWorldFrame = new Point3D();
        Point3D pointTwoInWorldFrame = new Point3D();

        for (LineSegment2D lineSegmentInSoleFrame : lineSegmentsInSoleFrame)
        {
           pointOneInWorldFrame.set(lineSegmentInSoleFrame.getFirstEndpointX(), lineSegmentInSoleFrame.getFirstEndpointY(), 0.0);
           pointTwoInWorldFrame.set(lineSegmentInSoleFrame.getSecondEndpointX(), lineSegmentInSoleFrame.getSecondEndpointY(), 0.0);

           soleTransform.transform(pointOneInWorldFrame);
           soleTransform.transform(pointTwoInWorldFrame);

           lineSegmentInWorldFrame.set(pointOneInWorldFrame.getX(), pointOneInWorldFrame.getY(), pointTwoInWorldFrame.getX(), pointTwoInWorldFrame.getY());

           ArrayList<PlanarRegion> intersectingRegionsToPack = new ArrayList<>();
           planarRegionsList.findPlanarRegionsIntersectingLineSegment(lineSegmentInWorldFrame, intersectingRegionsToPack);
           for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
           {
              ArrayList<Point2D[]> intersectionsInPlaneFrameToPack = new ArrayList<>();
              intersectingRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegmentInWorldFrame, intersectionsInPlaneFrameToPack);
              for (int i = 0; i < intersectionsInPlaneFrameToPack.size(); i++)
              {
                 Point2D[] points = intersectionsInPlaneFrameToPack.get(i);
                 for (int j = 0; j < points.length; j++)
                 {
                    Point2D point = points[j];
                    RigidBodyTransform regionTransformToWorld = new RigidBodyTransform();
                    intersectingRegion.getTransformToWorld(regionTransformToWorld);
                    Point3D pointInOriginalSoleFrame = new Point3D(point.getX(), point.getY(), 0.0);
                    regionTransformToWorld.transform(pointInOriginalSoleFrame);
                    inverseSoleTransform.transform(pointInOriginalSoleFrame);

                    if (pointInOriginalSoleFrame.getZ() > maxZInSoleFrame)
                    {
                       maxZInSoleFrame = pointInOriginalSoleFrame.getZ();
                       highestPointInSoleFrameToPack.set(pointInOriginalSoleFrame);
                       highestLineSegmentInSoleFrameToPack.set(lineSegmentInSoleFrame);
                    }
                 }
              }
           }
        }

        return maxZInSoleFrame;
     }

}
