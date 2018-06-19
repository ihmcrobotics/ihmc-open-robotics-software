package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionBasedPointFootSnapper implements PointFootSnapper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PointFootSnapperParameters parameters;

   private final Point2D snappedPoint2D = new Point2D();
   private final FramePoint3D snappedPoint3D = new FramePoint3D();
   private final ReferenceFrame planarRegionFrame;

   private final PlanarRegionsList planarRegionsList;

   private final RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
   private final ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D shrunkPolygon = new ConvexPolygon2D();
   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   public PlanarRegionBasedPointFootSnapper(PointFootSnapperParameters parameters)
   {
      this.parameters = parameters;
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
      double minimumPolygonArea = Math.PI * MathTools.square(parameters.distanceInsidePlanarRegion());

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         if (planarRegion.getNormal().getZ() > Math.cos(parameters.maximumNormalAngleFromVertical())
               && planarRegion.getConvexHull().getArea() > minimumPolygonArea)
         {
            this.planarRegionsList.addPlanarRegion(planarRegion);
         }
      }
   }

   @Override
   public Point3DReadOnly snapStep(double xPosition, double yPosition, double minimumZPosition)
   {
      List<PlanarRegion> intersectingRegions = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(xPosition, yPosition);
      if(intersectingRegions == null)
      {
         PlanarRegion closestRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(xPosition, yPosition);
         return projectPointIntoPlanarRegion(xPosition, yPosition, closestRegion, parameters.distanceInsidePlanarRegion());
      }
      else
      {
         double maxZ = Double.NEGATIVE_INFINITY;
         int highestPlanarRegionIndex = -1;
         for (int i = 0; i < intersectingRegions.size(); i++)
         {
            double regionHeight = intersectingRegions.get(i).getPlaneZGivenXY(xPosition, yPosition);
            if(regionHeight > maxZ)
            {
               highestPlanarRegionIndex = i;
               maxZ = regionHeight;
            }
         }

         return projectPointIntoPlanarRegion(xPosition, yPosition, intersectingRegions.get(highestPlanarRegionIndex), parameters.distanceInsidePlanarRegion());
      }
   }

   private Point3DReadOnly projectPointIntoPlanarRegion(double xPosition, double yPosition, PlanarRegion planarRegion, double distanceInside)
   {
      planarRegion.getTransformToWorld(planarRegionTransformToWorld);
      planarRegionFrame.update();
      planarRegionPolygon.set(planarRegion.getConvexHull());

      snappedPoint3D.setIncludingFrame(worldFrame, xPosition, yPosition, planarRegion.getPlaneZGivenXY(xPosition, yPosition));
      snappedPoint3D.changeFrame(planarRegionFrame);
      snappedPoint2D.set(snappedPoint3D);

      if (planarRegionPolygon.signedDistance(snappedPoint2D) > -distanceInside)
      {
         scaler.scaleConvexPolygon(planarRegionPolygon, distanceInside, shrunkPolygon);
         shrunkPolygon.orthogonalProjection(snappedPoint2D);
      }

      snappedPoint3D.set(snappedPoint2D);
      snappedPoint3D.changeFrame(worldFrame);
      return snappedPoint3D;
   }
}
