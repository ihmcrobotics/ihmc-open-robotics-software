package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface ScanPointFilter
{
   boolean test(int index, Point3DReadOnly point);

   public static ScanPointFilter combine(ScanPointFilter filterA, ScanPointFilter filterB)
   {
      return (index, point) -> filterA.test(index, point) && filterB.test(index, point);
   }
}