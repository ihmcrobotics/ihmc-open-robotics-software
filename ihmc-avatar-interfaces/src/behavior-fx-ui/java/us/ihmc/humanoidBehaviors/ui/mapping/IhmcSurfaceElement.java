package us.ihmc.humanoidBehaviors.ui.mapping;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

public class IhmcSurfaceElement
{
   private final double resolution;
   private final Plane3D plane = new Plane3D();

   private PlanarRegion mergeablePlanarRegion = new PlanarRegion();

   public IhmcSurfaceElement(double octreeResolution)
   {
      resolution = octreeResolution;
   }

   public IhmcSurfaceElement(IhmcSurfaceElement other)
   {
      resolution = other.resolution;
      plane.set(other.plane);
      mergeablePlanarRegion.set(other.mergeablePlanarRegion);
   }

   public void setPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      plane.set(pointOnPlane, planeNormal);
   }

   public void setPlane(Plane3D other)
   {
      plane.set(other.getPoint(), other.getNormal());
   }

   public void setMergeablePlanarRegion(PlanarRegion planarRegion)
   {
      mergeablePlanarRegion.set(planarRegion);
      mergeablePlanarRegion.setRegionId(planarRegion.getRegionId());
   }

   public double getResolution()
   {
      return resolution;
   }

   public Plane3D getPlane()
   {
      return plane;
   }

   public int getMergeablePlanarRegionId()
   {
      return mergeablePlanarRegion.getRegionId();
   }

   public Vector3DReadOnly getMergeablePlanarRegionNormal()
   {
      return mergeablePlanarRegion.getNormal();
   }

   public double getDistance(double positionWeight, double angleWeight)
   {
      Plane3D planarRegionPlane = mergeablePlanarRegion.getPlane();
      double positionDistance = planarRegionPlane.distance(plane.getPoint());
      double angleDistance = 1 - Math.abs(mergeablePlanarRegion.getNormal().dot(plane.getNormal()));

      return positionWeight * positionDistance + angleWeight * angleDistance;
   }

   public boolean isInPlanarRegion()
   {
      return mergeablePlanarRegion.isPointInside(plane.getPoint(), 0.005);
   }
}
