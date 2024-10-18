package us.ihmc.euclid.referenceFrame;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import static org.junit.jupiter.api.Assertions.*;

public class FramePlane3DTest
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
      FramePlane3D plane = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
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
      FramePlane3D plane = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
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
      FramePlane3D plane = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));

      point.set(3.0, 3.0, -4.0);
      plane.getPoint().set(1.0, 1.0, 1.0);
      plane.getNormal().set(2.0, 0.0, 0.0);
      expectedPoint.set(1.0, 3.0, -4.0);
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));
   }

	@Test
   public void testDistance()
   {
      FramePoint3D point = new FramePoint3D(worldFrame, 1.0, 1.0, 1.0);
      FramePlane3D plane = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      assertEquals(1.0, plane.distance(point), 1e-14);

      point.set(0.0, 0.0, 0.0);
      plane.getNormal().set(1.0, 1.0, 0.0);
      plane.getPoint().set(1.0, 1.0, 0.0);
      assertEquals(Math.sqrt(2), plane.distance(point), epsilon);
   }

	@Test
   public void testApplyTransform()
   {
      RigidBodyTransform transformation = new RigidBodyTransform();
      transformation.setRotationYawAndZeroTranslation(2.3);
      FramePlane3D plane = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane.applyTransform(transformation);
      FrameVector3D expectedNormal = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FramePoint3D expectedPoint = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
      assertTrue(plane.epsilonEquals(new FramePlane3D(expectedPoint, expectedNormal), epsilon));

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      FramePlane3D plane2 = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane2.applyTransform(transformation2);
      expectedNormal.set(0.0, 0.0, 1.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane2.epsilonEquals(new FramePlane3D(expectedPoint, expectedNormal), epsilon));

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.getTranslation().set(new Vector3D(1.0, 2.0, 3.0));
      FramePlane3D plane3 = new FramePlane3D(worldFrame, new Point3D(), new Vector3D(0.0, 0.0, 1.0));
      plane3.applyTransform(transformation3);
      expectedNormal.set(1.0, 0.0, 0.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane3.epsilonEquals(new FramePlane3D(expectedPoint, expectedNormal), epsilon));
   }

	@Test
   public void testIntersectionWithLine()
   {
	   FrameVector3D normal = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
	   FramePoint3D point = new FramePoint3D(worldFrame, 0.0, 0.0, 0.0);
	   FramePlane3D plane  = new FramePlane3D(point, normal);
	   
	   FramePoint3D origin = new FramePoint3D(worldFrame, 0.0, 1.0, -1.0);
	   FrameVector3D direction = new FrameVector3D(worldFrame, 1.0, 0.0, 1.0);
	   FrameLine3D line = new FrameLine3D(origin, direction);
	   
	   FramePoint3D pointToPack = new FramePoint3D(worldFrame);
	   plane.intersectionWith(line, pointToPack);
	   
	   FramePoint3D expectedIntersection = new FramePoint3D(worldFrame, 1.0, 1.0, 0.0);
	   assertTrue(pointToPack.epsilonEquals(expectedIntersection, epsilon));
   }
}
