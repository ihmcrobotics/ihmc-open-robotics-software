package us.ihmc.simulationconstructionset.physics.collision.simple;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.BoundingBox3d;

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
      Point3D minimumPoint = new Point3D();
      boundingBox.getMinPoint(minimumPoint);
      Point3D maximumPoint = new Point3D();
      boundingBox.getMaxPoint(maximumPoint);

      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(-0.2, -0.2, -0.3), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.2, 0.2, 0.3), maximumPoint, 1e-10);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(7.0, 8.0, 9.0);
      capsule.applyTransform(transform);

      capsule.getBoundingBox(boundingBox);
      boundingBox.getMinPoint(minimumPoint);
      boundingBox.getMaxPoint(maximumPoint);

      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(6.8, 7.8, 8.7), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(7.2, 8.2, 9.3), maximumPoint, 1e-10);
   }

}
