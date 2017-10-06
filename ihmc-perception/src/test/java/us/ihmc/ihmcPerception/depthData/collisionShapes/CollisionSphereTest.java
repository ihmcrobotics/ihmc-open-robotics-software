package us.ihmc.ihmcPerception.depthData.collisionShapes;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CollisionSphereTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testInsideSpherePoints()
   {
      Random random = new Random(98124L);

      CollisionSphere sphere = new CollisionSphere(new RigidBodyTransform(), 5.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = -5.0 + 10.0 * random.nextDouble();
         Vector3D vector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         Point3D point = new Point3D(vector);
         assertTrue(sphere.contains(point));
      }
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testOutsideSpherePoints()
   {
      Random random = new Random(1098551L);
      
      CollisionSphere sphere = new CollisionSphere(new RigidBodyTransform(), 5.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = (random.nextBoolean()?-1:1) * (5.0 + 10.0 * random.nextDouble());
         Vector3D vector = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         Point3D point = new Point3D(vector);
         assertFalse(sphere.contains(point));
      }
   }
   
   
}
