package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.ptcloud.alg.ApproximateSurfaceNormals;
import bubo.ptcloud.alg.PointVectorNN;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;
import georegression.geometry.UtilPlane3D_F64;
import georegression.metric.Distance3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import org.ddogleg.optimization.functions.FunctionNtoM;
import org.ddogleg.struct.FastQueue;
import us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitter.ScoringFunction;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

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
