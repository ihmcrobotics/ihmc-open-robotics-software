package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class FootstepCliffProximityCostCalculator
{
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepSnapperReadOnly snapper;
   private final FootstepPlannerParametersReadOnly parameters;
   private PlanarRegionsList planarRegionsList;

   public FootstepCliffProximityCostCalculator(FootstepPlannerParametersReadOnly parameters, FootstepSnapperReadOnly snapper, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public boolean isStepValid(DiscreteFootstep footstep)
   {
      double cliffHeightToAvoid = parameters.getCliffBaseHeightToAvoid();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      if(minimumDistanceFromCliffBottoms <= 0.0 || Double.isInfinite(cliffHeightToAvoid) || (cliffHeightToAvoid <= 0.0))
         return true;

      RigidBodyTransformReadOnly soleTransform = snapper.snapFootstep(footstep).getSnappedStepTransform(footstep);

      ArrayList<LineSegment2D> lineSegmentsInSoleFrame = new ArrayList<>();
      ConvexPolygon2D footPolygon = footPolygons.get(footstep.getRobotSide());
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly footPoint = footPolygon.getVertex(i);
         Point2D extendedPoint1 = new Point2D(footPoint);
         Point2D extendedPoint2 = new Point2D(footPoint);

         extendedPoint1.addX(minimumDistanceFromCliffBottoms * Math.signum(footPoint.getX()));
         extendedPoint2.addY(minimumDistanceFromCliffBottoms * Math.signum(footPoint.getY()));

         lineSegmentsInSoleFrame.add(new LineSegment2D(footPoint.getX(), footPoint.getY(), extendedPoint1.getX(), extendedPoint1.getY()));
         lineSegmentsInSoleFrame.add(new LineSegment2D(footPoint.getX(), footPoint.getY(), extendedPoint2.getX(), extendedPoint2.getY()));
      }

      Point3D highestPointInSoleFrame = new Point3D();
      LineSegment2D highestLineSegmentInSoleFrame = new LineSegment2D();

      double maximumCliffZInSoleFrame = findHighestPointInFrame(planarRegionsList, soleTransform, lineSegmentsInSoleFrame, highestPointInSoleFrame, highestLineSegmentInSoleFrame, new Point3D());

      boolean cliffDetected = maximumCliffZInSoleFrame >= cliffHeightToAvoid;
      return !cliffDetected;
   }

   public static double findHighestPointInFrame(PlanarRegionsList planarRegionsList, RigidBodyTransformReadOnly soleTransform, ArrayList<LineSegment2D> lineSegmentsInSoleFrame,
                                                Point3D highestPointInSoleFrameToPack, LineSegment2D highestLineSegmentInSoleFrameToPack, Point3D closestCliffPointToPack)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;
      double closestCliffPointDistance = Double.POSITIVE_INFINITY;

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
            List<Point2DBasics[]> intersectionsInPlaneFrameToPack = new ArrayList<>();
            intersectingRegion.getLineSegmentIntersectionsWhenProjectedVertically(lineSegmentInWorldFrame, intersectionsInPlaneFrameToPack);
            for (int i = 0; i < intersectionsInPlaneFrameToPack.size(); i++)
            {
               Point2DBasics[] points = intersectionsInPlaneFrameToPack.get(i);
               for (int j = 0; j < points.length; j++)
               {
                  Point2DBasics point = points[j];
                  RigidBodyTransform regionTransformToWorld = new RigidBodyTransform();
                  intersectingRegion.getTransformToWorld(regionTransformToWorld);
                  Point3D pointInOriginalSoleFrame = new Point3D(point.getX(), point.getY(), 0.0);
                  regionTransformToWorld.transform(pointInOriginalSoleFrame);
                  soleTransform.inverseTransform(pointInOriginalSoleFrame);

                  if(pointInOriginalSoleFrame.getZ() > 0.03 && pointInOriginalSoleFrame.distanceFromOrigin() < closestCliffPointDistance)
                  {
                     closestCliffPointDistance = pointInOriginalSoleFrame.distanceFromOrigin();
                     closestCliffPointToPack.set(pointInOriginalSoleFrame);
                  }

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
