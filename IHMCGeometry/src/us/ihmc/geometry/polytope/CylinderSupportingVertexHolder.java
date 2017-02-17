package us.ihmc.geometry.polytope;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

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
      this.transform.preMultiply(transform);
   }

   private final Vector3D tempVectorForSupportingVertex = new Vector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   @Override
   public Point3D getSupportingVertex(Vector3D supportDirection)
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

      Point3D supportingVertex = new Point3D(tempVectorForSupportingVertex);

      transform.transform(supportingVertex);
      return supportingVertex;
   }
   
   public String toString()
   {
      String string = "radius = " + radius + ", height = " + height + ", transform = " + transform;
      
      return string;
   }

}
