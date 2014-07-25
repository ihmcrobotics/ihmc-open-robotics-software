package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.HashMap;

import javax.vecmath.Point3d;

public class ResolutionFilter
{
   private double resolution;
   private final HashMap<Long, Point3d> map;

   final int bits = 20;
   final int mask = 0xFFFFFFFF >>> bits;
   final int offset = 1 << (bits - 1);

   public ResolutionFilter(double resolution)
   {
      this.resolution = resolution;
      map = new HashMap<>();
   }

   public boolean add(Point3d p)
   {
      Long hash = new Long(hash(p, resolution));

      if (map.containsKey(hash))
         return false;

      map.put(hash, p);
      return true;
   }

   public void clear()
   {
      map.clear();
   }

   private long hash(Point3d p, double res)
   {
      long hash = 0;
      hash += (((int) (p.x / res) + offset) & mask) << 2 * bits;
      hash += (((int) (p.y / res) + offset) & mask) << 1 * bits;
      hash += (((int) (p.z / res) + offset) & mask) << 0 * bits;
      return hash;
   }
}
