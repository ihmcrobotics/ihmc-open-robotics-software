package us.ihmc.simulationconstructionset.physics.collision.simple;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

public class CylinderShapeDescriptionTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProjectionWhenNotTransformed()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      assertEquals(radius, cylinder.getRadius(), 1e-10);
      assertEquals(height, cylinder.getHeight(), 1e-10);
      assertEquals(0.0, cylinder.getSmoothingRadius(), 1e-10);

      RigidBodyTransform transformCheck = new RigidBodyTransform();
      cylinder.getTransform(transformCheck);
      assertTrue(transformCheck.epsilonEquals(new RigidBodyTransform(), 1e-10));

      Point3d pointToProject = new Point3d(0.0, 0.0, 100.0);
      Point3d closestPointOnCylinder = new Point3d();
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(0.0, 0.0, -100.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, -height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(100.0, 0.0, 10.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(new Point3d(radius, 0.0, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(0.0, -20.0, 10.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, -radius, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(10.0, 10.0, height * 0.1);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(new Point3d(radius * Math.sqrt(2.0) / 2.0, radius * Math.sqrt(2.0) / 2.0, height * 0.1), closestPointOnCylinder, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProjectionWhenRotated()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      RigidBodyTransform transform = new RigidBodyTransform();

      double angle = Math.PI / 7.0;
      transform.applyRotationY(angle);
      cylinder.setTransform(transform);

      RigidBodyTransform transformCheck = new RigidBodyTransform(transform);
      cylinder.getTransform(transformCheck);
      assertTrue(transformCheck.epsilonEquals(transform, 1e-10));

      Point3d expectedPoint = new Point3d(-radius, 0.0, height / 2.0);
      transform.transform(expectedPoint);

      Point3d pointToProject = new Point3d(0.0, 0.0, 100.0);
      Point3d closestPointOnCylinder = new Point3d();
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(expectedPoint, closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(0.0, 0.0, -100.0);
      expectedPoint = new Point3d(radius, 0.0, -height / 2.0);
      transform.transform(expectedPoint);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(expectedPoint, closestPointOnCylinder, 1e-7);

      pointToProject = new Point3d(100.0, 0.0, 10.0);
      expectedPoint = new Point3d(radius, 0.0, height / 2.0);
      transform.transform(expectedPoint);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      JUnitTools.assertTuple3dEquals(expectedPoint, closestPointOnCylinder, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoundingBox()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      BoundingBox3d boundingBox = new BoundingBox3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      cylinder.getBoundingBox(boundingBox);

      Point3d minimumPoint = new Point3d();
      boundingBox.getMinPoint(minimumPoint);
      Point3d maximumPoint = new Point3d();
      boundingBox.getMaxPoint(maximumPoint);

      JUnitTools.assertTuple3dEquals(new Point3d(-0.5, -0.5, -0.05), minimumPoint, 1e-10);
      JUnitTools.assertTuple3dEquals(new Point3d(0.5, 0.5, 0.05), maximumPoint, 1e-10);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(Math.PI / 4.0, 0.0, 0.0);
      cylinder.applyTransform(transform);

      cylinder.getBoundingBox(boundingBox);

      boundingBox.getMinPoint(minimumPoint);
      boundingBox.getMaxPoint(maximumPoint);

      JUnitTools.assertTuple3dEquals(new Point3d(-0.5, -0.38890872965260115, -0.3889087296526011), minimumPoint, 1e-10);
      JUnitTools.assertTuple3dEquals(new Point3d(0.5, 0.38890872965260115, 0.3889087296526011), maximumPoint, 1e-10);
   }

   public static void main(String[] args)
   {
      String targetTests = CylinderShapeDescriptionTest.class.getName();
      String targetClassesInSamePackage = CylinderShapeDescription.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
