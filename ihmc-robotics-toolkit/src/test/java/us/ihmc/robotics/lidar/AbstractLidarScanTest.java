package us.ihmc.robotics.lidar;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class AbstractLidarScanTest
{
   Random rand = new Random(1098L);
   private static final double eps = 1e-7;

	@Test
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
         EuclidCoreTestTools.assertEquals(new Point3D(ranges[i], 0, 0), lidarScan.getPoint(i), eps);
      }
   }

	@Test
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
         Point3D p = new Point3D(ranges[i], 0, 0);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(params.sweepYawMin + i * sweepPerStep);
         transform.transform(p);
         EuclidCoreTestTools.assertEquals(p, lidarScan.getPoint(i), eps);
      }
   }

	@Test
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
         Point3D p = new Point3D(ranges[i], 0, 0);
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(sweepMin + i * sweepPerStep);
         transform.transform(p);
         EuclidCoreTestTools.assertEquals(p, lidarScan.getPoint(i), eps);
      }
   }

   public RigidBodyTransform randomTransform()
   {
      Quaternion rotate = new Quaternion(rand.nextDouble(), rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      Vector3D translate = new Vector3D(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());

      return new RigidBodyTransform(rotate, translate);
   }

	

	@Test
   public void testLineSegment()
   {
      // TODO Write test case
   }

	@Test
   public void testRay()
   {
      // TODO Write test case
   }
}
