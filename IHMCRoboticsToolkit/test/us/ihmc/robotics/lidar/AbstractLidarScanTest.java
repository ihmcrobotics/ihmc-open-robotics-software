package us.ihmc.robotics.lidar;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;

public class AbstractLidarScanTest
{
   Random rand = new Random(1098L);
   private static final double eps = 1e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIdentityScan()
   {
      int numPoints = 100;
      LidarScanParameters params = new LidarScanParameters(numPoints, 0, 0, 0, 0, 0, 0);

      float[] ranges = new float[numPoints];
      for (int i = 0; i < ranges.length; i++)
      {
         ranges[i] = rand.nextFloat();
      }

      LidarScan lidarScan = new LidarScan(params, new RigidBodyTransform(), new RigidBodyTransform(), ranges);

      for (int i = 0; i < ranges.length; i++)
      {
         JUnitTools.assertTuple3dEquals(new Point3d(ranges[i], 0, 0), lidarScan.getPoint(i), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleScanRotating()
   {
      int numPoints = 100;

      LidarScanParameters params = new LidarScanParameters(numPoints, -1, 1, 0, 0, 0, 0);
      RigidBodyTransform startTransform = new RigidBodyTransform();    // identity
      RigidBodyTransform endTransform = new RigidBodyTransform();    // identity

      float[] ranges = new float[numPoints];
      for (int i = 0; i < ranges.length; i++)
      {
         ranges[i] = rand.nextFloat();
      }

      LidarScan lidarScan = new LidarScan(params, startTransform, endTransform, ranges);

      double sweepPerStep = (params.sweepYawMax - params.sweepYawMin) / (params.pointsPerSweep - 1);
      for (int i = 0; i < ranges.length; i++)
      {
         Point3d p = new Point3d(ranges[i], 0, 0);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(params.sweepYawMin + i * sweepPerStep);
         transform.transform(p);
         JUnitTools.assertTuple3dEquals(p, lidarScan.getPoint(i), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInterpolationWhileRotating()
   {
      int numPoints = 100;
      double sweepMin = -1;
      double sweepMax = 1.1;


      LidarScanParameters params = new LidarScanParameters(numPoints, 0, 0, 0, 0, 0, 0);
      RigidBodyTransform startTransform = new RigidBodyTransform();
      RigidBodyTransform endTransform = new RigidBodyTransform();
      startTransform.setRotationYawAndZeroTranslation(sweepMin);
      endTransform.setRotationYawAndZeroTranslation(sweepMax);

      float[] ranges = new float[numPoints];
      for (int i = 0; i < ranges.length; i++)
      {
         ranges[i] = rand.nextFloat();
      }

      LidarScan lidarScan = new LidarScan(params, startTransform, endTransform, ranges);

      double sweepPerStep = (sweepMax - sweepMin) / (numPoints - 1);
      for (int i = 0; i < ranges.length; i++)
      {
         Point3d p = new Point3d(ranges[i], 0, 0);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(sweepMin + i * sweepPerStep);
         transform.transform(p);
         JUnitTools.assertTuple3dEquals(p, lidarScan.getPoint(i), eps);
      }
   }

   public RigidBodyTransform randomTransform()
   {
      Quat4d rotate = new Quat4d(rand.nextDouble(), rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      Vector3d translate = new Vector3d(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());

      return new RigidBodyTransform(rotate, translate);
   }

	

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLineSegment()
   {
      // TODO Write test case
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRay()
   {
      // TODO Write test case
   }
}
