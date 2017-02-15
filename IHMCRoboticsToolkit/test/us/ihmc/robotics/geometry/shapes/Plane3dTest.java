package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;

public class Plane3dTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Point3d point = new Point3d();
      Vector3d normal = new Vector3d(1.0, 2.0, 3.0);
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
      Point3d Q = new Point3d(0.0, 0.0, 2.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3d(0.0, 0.0, 0.0);
      assertTrue(plane.isOnOrAbove(Q));
      Q = new Point3d(0.0, 0.0, -2.0);
      assertFalse(plane.isOnOrAbove(Q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsOnOrBelow()
   {
      Plane3d plane = new Plane3d();
      Point3d Q = new Point3d(0.0, 0.0, -2.0);
      assertTrue(plane.isOnOrBelow(Q));
      Q = new Point3d(0.0, 0.0, 2.0);
      assertFalse(plane.isOnOrBelow(Q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjection()
   {
      Point3d q = new Point3d(1.0, 2.0, -3.0);
      Point3d v = new Point3d(1.0, 2.0, 0.0);
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
      Point3d point = new Point3d(1.0, 2.0, -3.0);
      Vector3d normal = new Vector3d(0.2, 1.7, 0.4);
      Plane3d plane = new Plane3d(point, normal);

      double x = 2.33;
      double y = 1.97;

      double z = plane.getZOnPlane(x, y);

      Point3d testPoint = new Point3d(x, y, z);
      assertTrue(plane.distance(testPoint) < 1e-10);

      normal = new Vector3d(0.2, 1.7, 0.0);
      plane = new Plane3d(point, normal);

      z = plane.getZOnPlane(x, y);
      assertTrue(Double.isNaN(z));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDistance()
   {
      Point3d q = new Point3d(1.0, 1.0, 1.0);
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
      JUnitTools.assertTuple3dEquals(plane.getNormalCopy(), new Vector3d(0.0, 0.0, 1.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane.getPointCopy(), new Point3d(0.0, 0.0, 0.0), 1e-14);

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      Plane3d plane2 = new Plane3d();
      plane2.applyTransform(transformation2);
      JUnitTools.assertTuple3dEquals(plane2.getNormalCopy(), new Vector3d(0.0, 0.0, 1.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane2.getPointCopy(), new Point3d(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      Plane3d plane3 = new Plane3d();
      plane3.applyTransform(transformation3);
      JUnitTools.assertTuple3dEquals(plane3.getNormalCopy(), new Vector3d(1.0, 0.0, 0.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane3.getPointCopy(), new Point3d(1.0, 2.0, 3.0), 1e-14);

      RigidBodyTransform transformation4 = new RigidBodyTransform();
      transformation4.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation4.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      Plane3d plane4 = new Plane3d();
      Plane3d plane5 = plane4.applyTransformCopy(transformation4);
      JUnitTools.assertTuple3dEquals(plane4.getNormalCopy(), new Vector3d(0.0, 0.0, 1.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane4.getPointCopy(), new Point3d(0.0, 0.0, 0.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane5.getNormalCopy(), new Vector3d(1.0, 0.0, 0.0), 1e-14);
      JUnitTools.assertTuple3dEquals(plane5.getPointCopy(), new Point3d(1.0, 2.0, 3.0), 1e-14);
   }
   

}
