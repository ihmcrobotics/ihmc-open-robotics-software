package us.ihmc.ihmcPerception.iterativeClosestPoint;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class KDNode
{

   public Point3D point3d;
   public int level;
   public int id;

   KDNode right;
   KDNode left;
   KDNode parent;

   public KDNode(double x, double y, double z, int level, int id)
   {
      this.point3d.setX(x);
      this.point3d.setY(y);
      this.point3d.setZ(z);
      this.level = level;
      this.id = id;
   }

   public KDNode(Point3D point, int level, int id)
   {
      this.point3d = point;
      this.level = level;
      this.id = id;
   }

   public KDNode(KDNode other)
   {
      this.point3d = other.point3d;
      this.level = other.level;
      this.id = other.id;
   }

   public double getX()
   {
      return point3d.getX();
   }

   public double getY()
   {
      return point3d.getY();
   }

   public double getZ()
   {
      return point3d.getZ();
   }

   public int getLevel()
   {
      return level;
   }

   public double distTo(KDNode other)
   {
      Point3D p = new Point3D(this.point3d.getX(), this.point3d.getY(), this.point3d.getZ());
      Point3D q = new Point3D(other.point3d.getX(), other.point3d.getY(), other.point3d.getZ());
      Vector3D pq = new Vector3D();
      pq.sub(p, q);
      return pq.length();
   }

   public double perpendicularDistance(KDNode other)
   {

      if (this.level % 3 == 0)
      {
         return Math.abs(other.getX() - this.getX());
      }
      else if (this.level % 3 == 1)
      {
         return Math.abs(other.getY() - this.getY());
      }
      else
      {
         return Math.abs(other.getZ() - this.getZ());
      }
   }

   public int compareTo(KDNode other)
   {
      if (other.point3d.getX() == this.point3d.getX() && other.point3d.getY() == this.point3d.getY() && other.point3d.getZ() == this.point3d.getZ())
      {
         return 0;
      }
      if (other.level % 3 == 0)
      {
         return (this.point3d.getX() < other.point3d.getX() ? -1 : 1);
      }
      if (other.level % 3 == 1)
      {
         return (this.point3d.getY() < other.point3d.getY() ? -1 : 1);
      }
      if (other.level % 3 == 2)
      {
         return (this.point3d.getZ() < other.point3d.getZ() ? -1 : 1);
      }
      return 0;
   }

   public String toString()
   {
      return "(" + getX() + "," + getY() + "," + getZ() + ")";
   }
}
