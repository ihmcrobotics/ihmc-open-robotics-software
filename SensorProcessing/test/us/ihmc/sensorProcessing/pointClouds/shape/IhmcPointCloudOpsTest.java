package us.ihmc.sensorProcessing.pointClouds.shape;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import georegression.geometry.UtilPlane3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import bubo.clouds.detect.tools.PointCloudShapeTools;

/**
 * @author Peter Abeles
 */
public class IhmcPointCloudOpsTest
{
   Random rand = new Random(234);

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void convert()
   {
      PlaneNormal3D_F64 orig = new PlaneNormal3D_F64(1,2,3,4,5,6);

      List<Point3D_F64> points = new ArrayList<Point3D_F64>();
      for( int i = 0; i < 100; i++ ) {
         points.add(PointCloudShapeTools.createPt(orig, rand.nextGaussian() * 2, rand.nextGaussian() * 2));
      }

      PlaneGeneral3D_F64 general = UtilPlane3D_F64.convert(orig,null);
      PlaneNormal3D_F64 found = new PlaneNormal3D_F64();
      IhmcPointCloudOps.convert(general,points,found);

      assertTrue(UtilPlane3D_F64.equals(orig,found,1e-8));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void adjustBoxNormals()
   {
      PlaneNormal3D_F64 a = new PlaneNormal3D_F64(1,0,0,1,0,0);
      PlaneNormal3D_F64 b = new PlaneNormal3D_F64(0,1,0,0,1,0);
      PlaneNormal3D_F64 c = new PlaneNormal3D_F64(0,0,1,0,0,1);

      // should do nothing
      IhmcPointCloudOps.adjustBoxNormals(a,b,c);
      assertEquals(1,a.n.x,1e-8);
      assertEquals(1,b.n.y,1e-8);
      assertEquals(1,c.n.z,1e-8);

      // check sign correction
      a.n.x = -1;
      IhmcPointCloudOps.adjustBoxNormals(a,b,c);
      assertEquals(1,a.n.x,1e-8);
      assertEquals(1,b.n.y,1e-8);
      assertEquals(1,c.n.z,1e-8);

      b.n.y = -1;
      IhmcPointCloudOps.adjustBoxNormals(a,b,c);
      assertEquals(1,a.n.x,1e-8);
      assertEquals(1,b.n.y,1e-8);
      assertEquals(1,c.n.z,1e-8);

      c.n.z = -1;
      IhmcPointCloudOps.adjustBoxNormals(a,b,c);
      assertEquals(1,a.n.x,1e-8);
      assertEquals(1,b.n.y,1e-8);
      assertEquals(1,c.n.z,1e-8);

      // check normalization
      a.n.x *= 2;
      b.n.y *= 3;
      b.n.z *= 4;

      IhmcPointCloudOps.adjustBoxNormals(a,b,c);
      assertEquals(1,a.n.x,1e-8);
      assertEquals(1,b.n.y,1e-8);
      assertEquals(1,c.n.z,1e-8);

   }
}
