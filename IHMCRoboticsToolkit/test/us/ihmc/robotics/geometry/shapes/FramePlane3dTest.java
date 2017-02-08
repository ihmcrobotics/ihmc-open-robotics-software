package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FrameLine;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FramePlane3dTest
{
	private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private static double epsilon = 1e-14;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsOnOrAbove()
   {
      FramePlane3d plane = new FramePlane3d(worldFrame);
      FramePoint q = new FramePoint(worldFrame, 0.0, 0.0, 2.0);
      assertTrue(plane.isOnOrAbove(q));
      q = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      assertTrue(plane.isOnOrAbove(q));
      q = new FramePoint(worldFrame, 0.0, 0.0, -2.0);
      assertFalse(plane.isOnOrAbove(q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsOnOrBelow()
   {
      FramePlane3d plane = new FramePlane3d(worldFrame);
      FramePoint q = new FramePoint(worldFrame, 0.0, 0.0, -2.0);
      assertTrue(plane.isOnOrBelow(q));
      q.set(0.0, 0.0, 1.0);
      assertFalse(plane.isOnOrBelow(q));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjection()
   {
      FramePoint point = new FramePoint(worldFrame, 1.0, 2.0, -3.0);
      FramePoint expectedPoint = new FramePoint(worldFrame, 1.0, 2.0, 0.0);
      FramePlane3d plane = new FramePlane3d(worldFrame);
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));

      point.set(3.0, 3.0, -4.0);
      plane.setPoint(1.0, 1.0, 1.0);
      plane.setNormal(2.0, 0.0, 0.0);
      expectedPoint.set(1.0, 3.0, -4.0);
      assertTrue(expectedPoint.epsilonEquals(plane.orthogonalProjectionCopy(point), 1e-14));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDistance()
   {
      FramePoint point = new FramePoint(worldFrame, 1.0, 1.0, 1.0);
      FramePlane3d plane = new FramePlane3d(worldFrame);
      assertEquals(1.0, plane.distance(point), 1e-14);

      point.set(0.0, 0.0, 0.0);
      plane.setNormal(1.0, 1.0, 0.0);
      plane.setPoint(1.0, 1.0, 0.0);
      assertEquals(Math.sqrt(2), plane.distance(point), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testApplyTransform()
   {
      RigidBodyTransform transformation = new RigidBodyTransform();
      transformation.setRotationYawAndZeroTranslation(2.3);
      FramePlane3d plane = new FramePlane3d(worldFrame);
      plane.applyTransform(transformation);
      FrameVector expectedNormal = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
      FramePoint expectedPoint = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
      assertTrue(plane.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation2 = new RigidBodyTransform();
      transformation2.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      FramePlane3d plane2 = new FramePlane3d(worldFrame);
      plane2.applyTransform(transformation2);
      expectedNormal.set(0.0, 0.0, 1.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane2.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation3 = new RigidBodyTransform();
      transformation3.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation3.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      FramePlane3d plane3 = new FramePlane3d(worldFrame);
      plane3.applyTransform(transformation3);
      expectedNormal.set(1.0, 0.0, 0.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane3.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));

      RigidBodyTransform transformation4 = new RigidBodyTransform();
      transformation4.setRotationPitchAndZeroTranslation(Math.PI / 2);
      transformation4.setTranslation(new Vector3d(1.0, 2.0, 3.0));
      FramePlane3d plane4 = new FramePlane3d(worldFrame);
      FramePlane3d plane5 = plane4.applyTransformCopy(transformation4);
      expectedNormal.set(0.0, 0.0, 1.0);
      expectedPoint.set(0.0, 0.0, 0.0);
      assertTrue(plane4.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));
      expectedNormal.set(1.0, 0.0, 0.0);
      expectedPoint.set(1.0, 2.0, 3.0);
      assertTrue(plane5.epsilonEquals(new FramePlane3d(expectedNormal, expectedPoint), epsilon));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionWithLine()
   {
	   FrameVector normal = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
	   FramePoint point = new FramePoint(worldFrame, 0.0, 0.0, 0.0);
	   FramePlane3d plane  = new FramePlane3d(normal, point);
	   
	   FramePoint origin = new FramePoint(worldFrame, 0.0, 1.0, -1.0);
	   FrameVector direction = new FrameVector(worldFrame, 1.0, 0.0, 1.0);
	   FrameLine line = new FrameLine(origin, direction);
	   
	   FramePoint pointToPack = new FramePoint(worldFrame);
	   plane.getIntersectionWithLine(pointToPack, line);
	   
	   FramePoint expectedIntersection = new FramePoint(worldFrame, 1.0, 1.0, 0.0);
	   assertTrue(pointToPack.epsilonEquals(expectedIntersection, epsilon));
   }
}
