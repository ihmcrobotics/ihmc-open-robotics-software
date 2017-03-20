package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.List;

import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;

public class CubeCalibration
{
   public static void orient(List<PlaneGeneral3D_F64> planes)
   {
      if (planes.size() < 3)
         return;

      LineParametric3D_F64 line = new LineParametric3D_F64();
      Point3D_F64 point = new Point3D_F64();
      Intersection3D_F64.intersect(planes.get(0), planes.get(1), line);
      Intersection3D_F64.intersect(planes.get(2), line, point);

      System.out.println(point);
   }
}
