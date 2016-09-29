package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CylinderShapeDescription<T extends CylinderShapeDescription<T>> implements CollisionShapeDescription<T>, SupportingVertexHolder
{
   private double radius;
   private double height;
   private double smoothingRadius = 0.0;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   public CylinderShapeDescription(double radius, double height)
   {
      this.radius = radius;
      this.height = height;
   }

   @Override
   public CylinderShapeDescription<T> copy()
   {
      CylinderShapeDescription<T> copy = new CylinderShapeDescription<T>(radius, height);
      copy.transform.set(this.transform);
      return copy;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public double getSmoothingRadius()
   {
      return smoothingRadius;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.multiply(transformToWorld, transform);
   }

   @Override
   public void setFrom(T cylinder)
   {
      this.radius = cylinder.getRadius();
      this.height = cylinder.getHeight();

      cylinder.getTransform(this.transform);
   }

   public SupportingVertexHolder getSupportingVertexHolder()
   {
      return this;
   }

   private final Vector3d tempVectorForSupportingVertex = new Vector3d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   @Override
   public Point3d getSupportingVertex(Vector3d supportDirection)
   {
      tempVectorForSupportingVertex.set(supportDirection);
      transform.transform(tempVectorForSupportingVertex);

      boolean up = tempVectorForSupportingVertex.getZ() > 0.0;

      tempVectorForSupportingVertex.setZ(0.0);
      double lengthSquared = tempVectorForSupportingVertex.lengthSquared();

      if (lengthSquared > 1e-10)
      {
         tempVectorForSupportingVertex.normalize();
      }
      else
      {
         tempVectorForSupportingVertex.set(1.0, 0.0, 0.0);
      }

      tempVectorForSupportingVertex.scale(radius);

      if (up)
      {
         tempVectorForSupportingVertex.setZ(height / 2.0);
      }
      else
      {
         tempVectorForSupportingVertex.setZ(-height / 2.0);
      }

      Point3d supportingVertex = new Point3d(tempVectorForSupportingVertex);

      tempTransform.set(transform);
      tempTransform.invert();
      tempTransform.transform(supportingVertex);
      return supportingVertex;
   }

}
