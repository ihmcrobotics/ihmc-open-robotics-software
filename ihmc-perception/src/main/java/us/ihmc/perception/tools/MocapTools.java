package us.ihmc.perception.tools;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class MocapTools
{
   public static void adjustMocapPositionsByOffset(ArrayList<Point3D> mocapPositionBuffer, Point3D origin)
   {
      Point3D offset = new Point3D(mocapPositionBuffer.get(0).getX(), -mocapPositionBuffer.get(0).getZ(), mocapPositionBuffer.get(0).getY());
      offset.sub(origin);
      LogTools.info("Offset: {}", offset);
      for (Point3D point : mocapPositionBuffer)
      {
         point.set(point.getX(), -point.getZ(), point.getY());
         point.sub(offset);
      }
   }
}
