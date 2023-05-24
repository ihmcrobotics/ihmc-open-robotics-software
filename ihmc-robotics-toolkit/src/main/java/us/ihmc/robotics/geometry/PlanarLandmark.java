package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.robotics.RegionInWorldInterface;

/**
 * This class is a limited version of a PlanarRegion with only the plane parameters for fast plane registration algorithms. It is used in the SKIPR
 * planar region mapping algorithm for fast registration using the Iterative Quaternion Averaging (IQA) approach. The lightweight nature of this class
 * arises from not having to track and process the concave hull of the region.
 * */
public class PlanarLandmark implements RegionInWorldInterface<PlanarLandmark>
{
   private int id;
   private double area = 0.0;

   /**
    * The parameters of the plane that this landmark lives in.
    * */
   private final Plane3D plane;

   /**
    * Transforms to go between the local frame of the landmark and the world frame.
    * */
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   /**
    * The bounding box of the landmark in the world frame.
    * */
   private final BoundingBox3D boundingBox3dInWorld = new BoundingBox3D(new Point3D(Double.NaN, Double.NaN, Double.NaN),
                                                                        new Point3D(Double.NaN, Double.NaN, Double.NaN));

   public PlanarLandmark(int id, Plane3D plane, BoundingBox3D boundingBox3dInWorld, RigidBodyTransformReadOnly fromLocalToWorldTransform, double area)
   {
      this.id = id;
      this.plane = plane;
      this.boundingBox3dInWorld.set(boundingBox3dInWorld);
      this.fromLocalToWorldTransform.set(fromLocalToWorldTransform);
      this.fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      this.area = area;
   }

   public PlanarLandmark(PlanarLandmark other)
   {
      this.id = other.id;
      this.plane = new Plane3D(other.plane);
      this.area = other.area;
      this.fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      this.fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
      this.boundingBox3dInWorld.set(other.boundingBox3dInWorld);
   }

   @Override
   public int getRegionId()
   {
      return id;
   }

   @Override
   public RigidBodyTransformReadOnly getTransformToLocal()
   {
      return fromWorldToLocalTransform;
   }

   @Override
   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox3dInWorld()
   {
      return boundingBox3dInWorld;
   }

   @Override
   public Point3DReadOnly getPoint()
   {
      return plane.getPoint();
   }

   @Override
   public UnitVector3DReadOnly getNormal()
   {
      return plane.getNormal();
   }

   @Override
   public void set(PlanarLandmark other)
   {
      this.plane.set(other.plane);
      this.area = other.area;
      this.fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      this.fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
      this.boundingBox3dInWorld.set(other.boundingBox3dInWorld);
      this.id = other.id;
   }

   @Override
   public boolean isPointInside(double xInLocal, double yInLocal)
   {
      return false;
   }

   public void applyTransform(RigidBodyTransformReadOnly transform)
   {
      plane.applyTransform(transform);
      fromLocalToWorldTransform.multiply(transform);
      fromWorldToLocalTransform.multiply(transform);

      // TODO: Fix this by creating a world frame bounding box from the vertices of the existing bounding box
      //boundingBox3dInWorld.applyTransform(transform);
   }
}
