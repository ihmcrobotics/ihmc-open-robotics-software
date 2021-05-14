package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
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

public class PlanarRegionCliffAvoider
{
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepSnapperReadOnly snapper;
   private final FootstepPlannerParametersReadOnly parameters;
   private PlanarRegionsList planarRegionsList;

   public PlanarRegionCliffAvoider(FootstepPlannerParametersReadOnly parameters, FootstepSnapperReadOnly snapper, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public boolean isStepValid(DiscreteFootstep footstep)
   {
      double cliffBottomHeightToAvoid = parameters.getCliffBaseHeightToAvoid();
      double minimumDistanceFromCliffBottoms = parameters.getMinimumDistanceFromCliffBottoms();

      double cliffTopHeightToAvoid = parameters.getCliffTopHeightToAvoid();
      double minimumDistanceFromCliffTops = parameters.getMinimumDistanceFromCliffTops();

      RigidBodyTransformReadOnly soleTransform = snapper.snapFootstep(footstep).getSnappedStepTransform(footstep);

      boolean checkCliffBottom = minimumDistanceFromCliffBottoms > 0.0 && cliffBottomHeightToAvoid > 0.0;
      if (checkCliffBottom)
      {
         ArrayList<LineSegment2D> lineSegmentsInSoleFrame = createLineSegmentsToProject(footstep, minimumDistanceFromCliffBottoms);
         double maxCliffHeight = findExtremumHeightInFrame(planarRegionsList, soleTransform, lineSegmentsInSoleFrame, true);
         if (maxCliffHeight > cliffBottomHeightToAvoid)
         {
            return false;
         }
      }

      boolean checkCliffTop = minimumDistanceFromCliffTops > 0.0 && cliffTopHeightToAvoid > 0.0;
      if (checkCliffTop)
      {
         ArrayList<LineSegment2D> lineSegmentsInSoleFrame = createLineSegmentsToProject(footstep, minimumDistanceFromCliffTops);
         double minCliffHeight = findExtremumHeightInFrame(planarRegionsList, soleTransform, lineSegmentsInSoleFrame, false);
         if (minCliffHeight < -cliffTopHeightToAvoid)
         {
            return false;
         }
      }

      return true;
   }

   private ArrayList<LineSegment2D> createLineSegmentsToProject(DiscreteFootstep footstep, double distance)
   {
      ArrayList<LineSegment2D> lineSegmentsInSoleFrame = new ArrayList<>();
      ConvexPolygon2D footPolygon = footPolygons.get(footstep.getRobotSide());
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly footPoint = footPolygon.getVertex(i);
         Point2D extendedPoint1 = new Point2D(footPoint);
         Point2D extendedPoint2 = new Point2D(footPoint);

         extendedPoint1.addX(distance * Math.signum(footPoint.getX()));
         extendedPoint2.addY(distance * Math.signum(footPoint.getY()));

         lineSegmentsInSoleFrame.add(new LineSegment2D(footPoint.getX(), footPoint.getY(), extendedPoint1.getX(), extendedPoint1.getY()));
         lineSegmentsInSoleFrame.add(new LineSegment2D(footPoint.getX(), footPoint.getY(), extendedPoint2.getX(), extendedPoint2.getY()));
      }
      return lineSegmentsInSoleFrame;
   }

   private double findExtremumHeightInFrame(PlanarRegionsList planarRegionsList, RigidBodyTransformReadOnly soleTransform, List<LineSegment2D> lineSegmentsInSoleFrame, boolean findMaximumHeight)
   {
      double extremumHeight = findMaximumHeight ? Double.NEGATIVE_INFINITY : Double.POSITIVE_INFINITY;

      LineSegment2D lineSegmentInWorldFrame = new LineSegment2D();
      Point3D pointOneInWorldFrame = new Point3D();
      Point3D pointTwoInWorldFrame = new Point3D();

      for (LineSegment2DReadOnly lineSegmentInSoleFrame : lineSegmentsInSoleFrame)
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

                  if (findMaximumHeight)
                  {
                     if (pointInOriginalSoleFrame.getZ() > extremumHeight)
                     {
                        extremumHeight = pointInOriginalSoleFrame.getZ();
                     }
                  }
                  else
                  {
                     if (pointInOriginalSoleFrame.getZ() < extremumHeight)
                     {
                        extremumHeight = pointInOriginalSoleFrame.getZ();
                     }
                  }
               }
            }
         }
      }

      return extremumHeight;
   }

}
