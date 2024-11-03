package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FramePlane3dTest
{
	private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private static double epsilon = 1e-14;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testIsOnOrAbove()
   {
      FramePlane3d plane = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      FramePoint3D q = new FramePoint3D(worldFrame, 0.0, 0.0, 2.0);
      assertTrue(plane.isOnOrAbove(q));
      q = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      assertTrue(plane.isOnOrAbove(q));
      q = new FramePoint3D(worldFrame, 0.0, 0.0, -2.0);
      assertFalse(plane.isOnOrAbove(q));
   }

	@Test
   public void testIsOnOrBelow()
   {
      FramePlane3d plane = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      FramePoint3D q = new FramePoint3D(worldFrame, 0.0, 0.0, -2.0);
      assertTrue(plane.isOnOrBelow(q));
      q.set(0.0, 0.0, 1.0);
      assertFalse(plane.isOnOrBelow(q));
   }

	@Test
   public void testOrthogonalProjection()
   {
      FramePoint3D point = new FramePoint3D(worldFrame, 1.0, 2.0, -3.0);
      FramePoint3D expectedPoint = new FramePoint3D(worldFrame, 1.0, 2.0, 0.0);
      FramePlane3d plane = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));

      point.set(3.0, 3.0, -4.0);
      plane.setPoint(1.0, 1.0, 1.0);
      plane.setNormal(2.0, 0.0, 0.0);
      expectedPoint.set(1.0, 3.0, -4.0);
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));
   }

	@Test
   public void testDistance()
   {
      FramePoint3D point = new FramePoint3D(worldFrame, 1.0, 1.0, 1.0);
      FramePlane3d plane = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      assertEquals(1.0, plane.distance(point), 1e-14);

      point.set(0.0, 0.0, 0.0);
      plane.setNormal(1.0, 1.0, 0.0);
      plane.setPoint(1.0, 1.0, 0.0);
      assertEquals(Math.sqrt(2), plane.distance(point), epsilon);
   }

	@Test
   public void testApplyTransform()
   {
      RigidBodyTransform transformation = new RigidBodyTransform();
      transformation.setRotationYawAndZeroTranslation(2.3);
      FramePlane3d plane = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane.applyTransform(transformation);
      FrameVector3D expectedNormal = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FramePoint3D expectedPoint = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      assertTrue(plane.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      FramePlane3d plane2 = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane2.applyTransform(transformation2);
      expectedNormal.set(0.0, 0.0, 1.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane2.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      FramePlane3d plane3 = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane3.applyTransform(transformation3);
      expectedNormal.set(1.0, 0.0, 0.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane3.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation4 = new RigidBodyTransform();
      transformation4.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation4.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      FramePlane3d plane4 = new FramePlane3d(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      FramePlane3d plane5 = plane4.applyTransformCopy(transformation4);
      expectedNormal.set(0.0, 0.0, 1.0);
      expectedPoint.set(0.0, 0.0, 0.0);
      assertTrue(plane4.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));
      expectedNormal.set(1.0, 0.0, 0.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane5.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));
   }

	@Test
   public void testIntersectionWithLine()
   {
	   FrameVector3D normal = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
	   FramePoint3D point = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
	   FramePlane3d plane  = new FramePlane3d(normal, point);
	   
	   FramePoint3D origin = new FramePoint3D(worldFrame, 0.0, 1.0, -1.0);
	   FrameVector3D direction = new FrameVector3D(worldFrame, 1.0, 0.0, 1.0);
	   FrameLine3D line = new FrameLine3D(origin, direction);
	   
	   FramePoint3D pointToPack = new FramePoint3D(worldFrame);
	   plane.getIntersectionWithLine(pointToPack, line);
	   
	   FramePoint3D expectedIntersection = new FramePoint3D(worldFrame, 1.0, 1.0, 0.0);
	   assertTrue(pointToPack.epsilonEquals(expectedIntersection, epsilon));
   }
}
