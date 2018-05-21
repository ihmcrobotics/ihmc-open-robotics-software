package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionBasedPointFootSnapper implements PointFootSnapper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double distanceInside = 0.05;
   private static final double maxNormalAngleFromVertical = 0.3;


   private final Point2D unsnappedPoint = new Point2D();
   private final Point2D snappedPoint2D = new Point2D();
   private final Point3D snappedPoint = new Point3D();
   private final ReferenceFrame planarRegionFrame;

   private final PlanarRegionsList planarRegionsList;
   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);


   private final RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
   private final FrameConvexPolygon2D planarRegionPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D shrunkPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   public PlanarRegionBasedPointFootSnapper()
   {
      planarRegionsList = new PlanarRegionsList();
      planarRegionFrame = new ReferenceFrame("planarRegionFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(planarRegionTransformToWorld);
         }
      };
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.clear();
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         planarRegion.getNormal(planeNormal);

         double angle = planeNormal.angle(verticalAxis);

         if (angle < maxNormalAngleFromVertical)
         {
            this.planarRegionsList.addPlanarRegion(planarRegion);
         }
      }
   }

   @Override
   public Point3DReadOnly snapStep(double xPosition, double yPosition)
   {
      unsnappedPoint.set(xPosition, yPosition);
      return snapStep(unsnappedPoint);
   }

   public Point3DReadOnly snapStep(Point2DReadOnly unsnappedPoint)
   {
      List<PlanarRegion> intersectingRegions = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(unsnappedPoint);
      if(intersectingRegions == null)
      {
         PlanarRegion closestRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(unsnappedPoint);
         Point2DReadOnly snappedPoint2D = projectPointIntoPlanarRegion(unsnappedPoint, closestRegion, distanceInside);
         double zHeight = closestRegion.getPlaneZGivenXY(snappedPoint2D.getX(), snappedPoint2D.getY());

         snappedPoint.set(snappedPoint2D, zHeight);
      }
      else
      {
         double maxZ = Double.NEGATIVE_INFINITY;
         PlanarRegion planarRegion = null;
         for (int i = 0; i < intersectingRegions.size(); i++)
         {
            double regionHeight = intersectingRegions.get(i).getPlaneZGivenXY(unsnappedPoint.getX(), unsnappedPoint.getY());
            if(regionHeight > maxZ)
            {
               planarRegion = intersectingRegions.get(i);
               maxZ = regionHeight;
            }
         }

         Point2DReadOnly snappedPoint2D = projectPointIntoPlanarRegion(unsnappedPoint, planarRegion, distanceInside);
         snappedPoint.set(snappedPoint2D, maxZ);
      }

      return snappedPoint;
   }

   private Point2DReadOnly projectPointIntoPlanarRegion(Point2DReadOnly unsnappedPoint, PlanarRegion planarRegion, double distanceInside)
   {
      planarRegion.getTransformToWorld(planarRegionTransformToWorld);
      planarRegionFrame.update();
      planarRegionPolygon.setReferenceFrame(planarRegionFrame);
      planarRegionPolygon.set(planarRegion.getConvexHull());
      planarRegionPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      if (planarRegionPolygon.signedDistance(unsnappedPoint) > -distanceInside)
      {
         scaler.scaleConvexPolygon(planarRegionPolygon, distanceInside, shrunkPolygon);
         shrunkPolygon.orthogonalProjection(unsnappedPoint, snappedPoint2D);
      }
      else
      {
         snappedPoint2D.set(unsnappedPoint);
      }

      return snappedPoint2D;
   }
}
