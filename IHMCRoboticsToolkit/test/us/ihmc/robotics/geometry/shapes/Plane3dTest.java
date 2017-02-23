package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Plane3dTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Point3D point = new Point3D();
      Vector3D normal = new Vector3D(1.0, 2.0, 3.0);
      Plane3d plane = new Plane3d(point, normal);
      assertTrue(point.equals(plane.getPointCopy()));
      assertFalse(point == plane.getPointCopy());
      normal.normalize();
      assertTrue(normal.equals(plane.getNormalCopy()));
      Plane3d plane2 = new Plane3d(point,normal);
      assertTrue(plane2.epsilonEquals(plane, 1e-17));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEmptyConstructor()
   {
      Plane3d plane = new Plane3d();
      assertTrue(plane.getNormalCopy().getX() == 0.0);
      assertTrue(plane.getNormalCopy().getY() == 0.0);
      assertTrue(plane.getNormalCopy().getZ() == 1.0);
      assertTrue(plane.getPointCopy().getX() == 0.0);
      assertTrue(plane.getPointCopy().getY() == 0.0);
      assertTrue(plane.getPointCopy().getZ() == 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsOnOrAbove()
   {
      Plane3d plane = new Plane3d();
      Point3D Q = new Point3D(0.0, 0.0, 2.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3D(0.0, 0.0, 0.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3D(0.0, 0.0, -2.0);
      assertFalse(plane.isOnOrAbove(Q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsOnOrBelow()
   {
      Plane3d plane = new Plane3d();
      Point3D Q = new Point3D(0.0, 0.0, -2.0);
      assertTrue(plane.isOnOrBelow(Q));
      Q = new Point3D(0.0, 0.0, 2.0);
      assertFalse(plane.isOnOrBelow(Q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjection()
   {
      Point3D q = new Point3D(1.0, 2.0, -3.0);
      Point3D v = new Point3D(1.0, 2.0, 0.0);
      Plane3d plane = new Plane3d();
      assertTrue(v.equals(plane.orthogonalProjectionCopy(q)));

      q.set(3.0, 3.0, -4.0);
      plane.setPoint(1.0, 1.0, 1.0);
      plane.setNormal(2.0, 0.0, 0.0);
      v.set(1.0, 3.0, -4.0);
      assertTrue(v.equals(plane.orthogonalProjectionCopy(q)));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetZOnPlane()
   {
      Point3D point = new Point3D(1.0, 2.0, -3.0);
      Vector3D normal = new Vector3D(0.2, 1.7, 0.4);
      Plane3d plane = new Plane3d(point, normal);

      double x = 2.33;
      double y = 1.97;

      double z = plane.getZOnPlane(x, y);

      Point3D testPoint = new Point3D(x, y, z);
      assertTrue(plane.distance(testPoint) < 1e-10);

      normal = new Vector3D(0.2, 1.7, 0.0);
      plane = new Plane3d(point, normal);

      z = plane.getZOnPlane(x, y);
      assertTrue(Double.isNaN(z));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDistance()
   {
      Point3D q = new Point3D(1.0, 1.0, 1.0);
      Plane3d plane = new Plane3d();
      assertEquals(1.0, plane.distance(q), 1e-14);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform()
   {
      RigidBodyTransform transformation = new RigidBodyTransform();
      transformation.setRotationYawAndZeroTranslation(2.3);
      Plane3d plane = new Plane3d();
      plane.applyTransform(transformation);
      EuclidCoreTestTools.assertTuple3DEquals(plane.getNormalCopy(), new Vector3D(0.0, 0.0, 1.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane.getPointCopy(), new Point3D(0.0, 0.0, 0.0), 1e-14);

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.setTranslation(new Vector3D(1.0, 2.0, 3.0));
      Plane3d plane2 = new Plane3d();
      plane2.applyTransform(transformation2);
      EuclidCoreTestTools.assertTuple3DEquals(plane2.getNormalCopy(), new Vector3D(0.0, 0.0, 1.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane2.getPointCopy(), new Point3D(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.setTranslation(new Vector3D(1.0, 2.0, 3.0));
      Plane3d plane3 = new Plane3d();
      plane3.applyTransform(transformation3);
      EuclidCoreTestTools.assertTuple3DEquals(plane3.getNormalCopy(), new Vector3D(1.0, 0.0, 0.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane3.getPointCopy(), new Point3D(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation4 = new RigidBodyTransform();
      transformation4.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation4.setTranslation(new Vector3D(1.0, 2.0, 3.0));
      Plane3d plane4 = new Plane3d();
      Plane3d plane5 = plane4.applyTransformCopy(transformation4);
      EuclidCoreTestTools.assertTuple3DEquals(plane4.getNormalCopy(), new Vector3D(0.0, 0.0, 1.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane4.getPointCopy(), new Point3D(0.0, 0.0, 0.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane5.getNormalCopy(), new Vector3D(1.0, 0.0, 0.0), 1e-14);
      EuclidCoreTestTools.assertTuple3DEquals(plane5.getPointCopy(), new Point3D(1.0, 2.0, 3.0), 1e-14);
   }
   

}
