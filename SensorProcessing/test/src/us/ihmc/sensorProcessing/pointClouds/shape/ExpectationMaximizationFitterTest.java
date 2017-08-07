package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ExpectationMaximizationFitterTest
{
   static Random rand = new Random(152L);

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testFitToCube()
   {
      Point3D_F64 center = new Point3D_F64(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      List<Point3D_F64> points = createBoxCloud(center, 1000, 1, .003);

      List<PlaneGeneral3D_F64> planes = ExpectationMaximizationFitter.fit(3, rand, points, ExpectationMaximizationFitter.getGaussianSqauresMixedError(.003 / 2), 25);
      System.out.println(center);
      System.out.println(getIntersection(planes));

   }

   private static List<Point3D_F64> createBoxCloud(Point3D_F64 center, int numPoints, double size, double noise)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      noise /= 3; //gives 98% of gaussian noise within range

      Vector3D_F64[] sides = new Vector3D_F64[] { new Vector3D_F64(1, 1, 0), new Vector3D_F64(1, 0, 1), new Vector3D_F64(0, 1, 1) };

      for (int side = 0; side < sides.length; side++)
      {
         Vector3D_F64 sideVector = sides[side];
         for (int i = 0; i < numPoints; i++)
         {
            double x = (size * (rand.nextDouble() - .5) * sideVector.x) + (center.x - (sideVector.x * size / 2));
            double y = (size * (rand.nextDouble() - .5) * sideVector.y) + (center.y - (sideVector.y * size / 2));
            double z = (size * (rand.nextDouble() - .5) * sideVector.z) + (center.z - (sideVector.z * size / 2));

            x += ((rand.nextGaussian() - .5) * noise);
            y += ((rand.nextGaussian() - .5) * noise);
            z += ((rand.nextGaussian() - .5) * noise);

            cloud.add(new Point3D_F64(x, y, z));
         }
      }
      return cloud;
   }

   private static Point3D_F64 getIntersection(List<PlaneGeneral3D_F64> planes)
   {
      if (planes.size() < 3)
         assert (false);

      LineParametric3D_F64 line = new LineParametric3D_F64();
      Point3D_F64 point = new Point3D_F64();
      Intersection3D_F64.intersect(planes.get(0), planes.get(1), line);
      Intersection3D_F64.intersect(planes.get(2), line, point);

      return point;
   }

}
