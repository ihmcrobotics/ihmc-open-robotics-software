package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.LinkedHashMap;

import javax.vecmath.Point3d;

public class CappedResolutionFilter
{
   private double resolution;
   private final CapacityHashMap<Long, Point3d> map;

   final int bits = 20;
   final int mask = 0xFFFFFFFF >>> bits;
   final int offset = 1 << (bits - 1);

   public CappedResolutionFilter(double resolution, long capacity)
   {
      this.resolution = resolution;
      map = new CapacityHashMap<>(capacity);
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
   
   private static class CapacityHashMap<K, V> extends LinkedHashMap<K, V> {
      private static final long serialVersionUID = 1L;

      private long capacity;
      
      public CapacityHashMap(long capacity)
      {
         this.capacity = capacity;
      }
      
      @Override
      protected boolean removeEldestEntry(java.util.Map.Entry<K, V> eldest)
      {
         return size() > capacity;
      }
   }
}
