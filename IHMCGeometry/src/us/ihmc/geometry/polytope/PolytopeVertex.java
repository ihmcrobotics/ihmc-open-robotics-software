package us.ihmc.geometry.polytope;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PolytopeVertex
{
   private final Point3d position = new Point3d();
   private final ArrayList<PolytopeVertex> connectingVertices = new ArrayList<>();

   public PolytopeVertex(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public PolytopeVertex(Point3d position)
   {
      this.position.set(position);
   }

   public void addConnectingVertex(PolytopeVertex vertex)
   {
      if (!connectingVertices.contains(vertex))
         connectingVertices.add(vertex);
   }

   public int getNumberOfConnectingVertices()
   {
      return connectingVertices.size();
   }

   public Point3d getPosition()
   {
      return position;
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(position);
   }

   public double dot(Vector3d vector)
   {
      return position.getX() * vector.getX() + position.getY() * vector.getY() + position.getZ() * vector.getZ();
   }

}
