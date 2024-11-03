package us.ihmc.robotics.dataStructures;

import java.util.ArrayList;
import java.util.LinkedList;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;

public class DecayingResolutionFilter
{
   private double resolution;
   private long decayMillis;
   int capacity = 100000;

   private final TLongObjectHashMap<TimestampedPoint> map;
   //   private final HashMap<Long, TimestampedPoint> map;
   private final LinkedList<TimestampedPoint> list;

   public DecayingResolutionFilter(double resolution, long decayMillis, int maxSizeAllowed)
   {
      this.resolution = resolution;
      this.decayMillis = decayMillis;
      this.capacity = maxSizeAllowed;

      map = new TLongObjectHashMap<TimestampedPoint>();
      list = new LinkedList<TimestampedPoint>();
   }

   public boolean add(Point3D p)
   {
      return add(p.getX(), p.getY(), p.getZ());
   }

   public boolean add(double x, double y, double z)
   {
      return add((float) x, (float) y, (float) z);
   }

   public boolean add(float x, float y, float z)
   {
      long hash = hash(x, y, z, resolution);

      removeDecayedPoints();
      
      if (!map.containsKey(hash))
      {
         TimestampedPoint point = new TimestampedPoint(x, y, z, System.currentTimeMillis());
         synchronized (list)
         {
            atomicAddToMapAndList(hash, point);
         }
         return true;
      }

      return false;
   }

   private synchronized void atomicAddToMapAndList(long hash, TimestampedPoint point)
   {
      map.put(hash, point);
      list.addLast(point);
   }

   private void removeDecayedPoints()
   {
      TimestampedPoint timePoint;

      synchronized (list)
      {
         if (list.size() >= capacity && capacity > 0)
         {
            atomicRemoveFromListAndMap();
         }
      }

      long time = System.currentTimeMillis();
      while ((timePoint = list.peekFirst()) != null)
      {
         if (timePoint.timestamp + decayMillis > time || decayMillis<0)
         {
            return;
         }

         synchronized (list)
         {
            atomicRemoveFromListAndMap();
         }
      }
   }

   private synchronized void atomicRemoveFromListAndMap()
   {
      TimestampedPoint timePoint;
      timePoint = list.removeFirst();
      map.remove(hash(timePoint));
   }

   //================================================================================
   // Private Utility Functions
   //================================================================================

   private long hash(TimestampedPoint point)
   {
      return hash(point.getX32(), point.getY32(), point.getZ32(), resolution);
   }

   private static final int bits = 20;
   private static final int mask = 0xFFFFFFFF >>> bits;
   private static final int offset = 1 << (bits - 1);

   private long hash(float x, float y, float z, double res)
   {
      long hash = 0;
      hash += (((int) (x / res) + offset) & mask) << 2 * bits;
      hash += (((int) (y / res) + offset) & mask) << 1 * bits;
      hash += (((int) (z / res) + offset) & mask) << 0 * bits;

      return hash;
   }

   //================================================================================
   // Setters
   //================================================================================

   public void clear()
   {
      synchronized (list)
      {
         map.clear();
         list.clear();
      }
   }

   public void setResolution(double resolution)
   {
      this.resolution = resolution;
   }

   public void setDecay(long decayMillis)
   {
      this.decayMillis = decayMillis;
   }

   public void setCapacity(int capacity)
   {
      this.capacity = capacity;
   }

   //================================================================================
   // Getters
   //================================================================================

   /**
    * Get the points in a new array. The points them self are not copied.
    * @return new Point3f array
    */
   public Point3D32[] getPoints3f()
   {
      synchronized (list)
      {
         Point3D32[] points = new Point3D32[list.size()];

         for (int i = 0; i < list.size(); i++)
         {
            points[i] = list.get(i);
         }
         
         return points;
      }
   }

   /**
    * Unsafe when reading/writing from different threads, but it is quicker.
    * @return
    */
   public Point3D32[] getPoints3fUnsynchronized()
   {
      return list.toArray(new Point3D32[0]);
   }

   public ArrayList<TimestampedPoint> getPointsCopy()
   {
      // Thread-safely copies points into a new array
      ArrayList<TimestampedPoint> copy;

      synchronized (list)
      {
         copy = new ArrayList<TimestampedPoint>(list);
      }

      return copy;
   }

   public Point3D32 getNearestIntersection(Point3D32 origin, Vector3D32 direction)
   {
      ArrayList<TimestampedPoint> points = getPointsCopy();
      direction.normalize();
      float dx, dy, dz, dot;
      double distanceToLine, distance;

      double nearestDistance = Double.POSITIVE_INFINITY;
      Point3D32 nearestPoint = null;

      for (Point3D32 p : points)
      {
         dx = origin.getX32() - p.getX32();
         dy = origin.getY32() - p.getY32();
         dz = origin.getZ32() - p.getZ32();

         dot = dx * direction.getX32() + dy * direction.getY32() + dz * direction.getZ32();

         dx = dx - dot * direction.getX32();
         dy = dy - dot * direction.getY32();
         dz = dz - dot * direction.getZ32();

         distanceToLine = Math.sqrt(dx * dx + dy * dy + dz * dz);

         if (distanceToLine < resolution / 2)
         {
            distance = origin.distance(p);
            if (distance < nearestDistance)
            {
               nearestDistance = distance;
               nearestPoint = p;
            }
         }
      }

      return nearestPoint;
   }
}