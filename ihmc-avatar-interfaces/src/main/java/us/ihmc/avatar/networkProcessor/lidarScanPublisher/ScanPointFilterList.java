package us.ihmc.avatar.networkProcessor.lidarScanPublisher;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ScanPointFilterList implements ScanPointFilter
{
   private final List<ScanPointFilter> filters = new ArrayList<>();

   public ScanPointFilterList()
   {
   }

   public void addFilter(ScanPointFilter filter)
   {
      if (filter == this)
         throw new IllegalArgumentException("Cannot add this to itself.");

      filters.add(filter);
   }

   @Override
   public boolean test(int index, Point3DReadOnly point)
   {
      if (filters.isEmpty())
         return true;
      for (ScanPointFilter filter : filters)
      {
         if (!filter.test(index, point))
            return false;
      }

      return true;
   }
}
