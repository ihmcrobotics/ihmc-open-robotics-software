package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CylinderSupportingVertexHolder implements SupportingVertexHolder
{
   private double radius;
   private double height;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   public CylinderSupportingVertexHolder(double radius, double height)
   {
      this.radius = radius;
      this.height = height;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   public void setTransform(RigidBodyTransform transform)
   {
      this.transform.set(transform);
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      this.transform.multiply(transform, this.transform);
   }

   private final Vector3d tempVectorForSupportingVertex = new Vector3d();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   @Override
   public Point3d getSupportingVertex(Vector3d supportDirection)
   {
      tempTransform.set(transform);
      tempTransform.invert();
      
      tempVectorForSupportingVertex.set(supportDirection);
      tempTransform.transform(tempVectorForSupportingVertex);

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

      transform.transform(supportingVertex);
      return supportingVertex;
   }
   
   public String toString()
   {
      String string = "radius = " + radius + ", height = " + height + ", transform = " + transform;
      
      return string;
   }

}
