package us.ihmc.simulationconstructionset.physics.collision.simple;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;

public class CapsuleShapeDescriptionTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
      double radius = 0.2;
      double height = 0.6;

      CapsuleShapeDescription<?> capsule = new CapsuleShapeDescription<>(radius, height);

      BoundingBox3d boundingBox = new BoundingBox3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      capsule.getBoundingBox(boundingBox);
      Point3d minimumPoint = new Point3d();
      boundingBox.getMinPoint(minimumPoint);
      Point3d maximumPoint = new Point3d();
      boundingBox.getMaxPoint(maximumPoint);

      JUnitTools.assertTuple3dEquals(new Point3d(-0.2, -0.2, -0.3), minimumPoint, 1e-10);
      JUnitTools.assertTuple3dEquals(new Point3d(0.2, 0.2, 0.3), maximumPoint, 1e-10);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(7.0, 8.0, 9.0);
      capsule.applyTransform(transform);

      capsule.getBoundingBox(boundingBox);
      boundingBox.getMinPoint(minimumPoint);
      boundingBox.getMaxPoint(maximumPoint);

      JUnitTools.assertTuple3dEquals(new Point3d(6.8, 7.8, 8.7), minimumPoint, 1e-10);
      JUnitTools.assertTuple3dEquals(new Point3d(7.2, 8.2, 9.3), maximumPoint, 1e-10);
   }

}
