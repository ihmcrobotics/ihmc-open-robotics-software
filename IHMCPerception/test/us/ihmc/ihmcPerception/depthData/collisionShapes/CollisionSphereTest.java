package us.ihmc.ihmcPerception.depthData.collisionShapes;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CollisionSphereTest
{
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 3000)
   public void testInsideSpherePoints()
   {
      Random random = new Random(98124L);

      CollisionSphere sphere = new CollisionSphere(new RigidBodyTransform(), 5.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = -5.0 + 10.0 * random.nextDouble();
         Vector3d vector = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         Point3d point = new Point3d(vector);
         assertTrue(sphere.contains(point));
      }
   }
   
   @EstimatedDuration(duration = 0.1)
   @Test(timeout = 3000)
   public void testOutsideSpherePoints()
   {
      Random random = new Random(1098551L);
      
      CollisionSphere sphere = new CollisionSphere(new RigidBodyTransform(), 5.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = (random.nextBoolean()?-1:1) * (5.0 + 10.0 * random.nextDouble());
         Vector3d vector = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         Point3d point = new Point3d(vector);
         assertFalse(sphere.contains(point));
      }
   }
   
   
}
