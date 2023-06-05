package us.ihmc.perception.collisionShapes;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.depthData.collisionShapes.CollisionSphere;

public class CollisionSphereTest
{
   @Test
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
   
   @Test
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
