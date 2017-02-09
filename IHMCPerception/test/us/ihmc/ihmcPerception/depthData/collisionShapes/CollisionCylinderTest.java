package us.ihmc.ihmcPerception.depthData.collisionShapes;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CollisionCylinderTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testInsideCylinderPoints()
   {
      Random random = new Random(98124L);

      CollisionCylinder cylinder = new CollisionCylinder(new RigidBodyTransform(), 5.0, 10.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = -5.0 + 10.0 * random.nextDouble();
         Vector2d vector = new Vector2d(random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         double z = -5.0 +  10.0 * random.nextDouble();  
         
         
         Point3d point = new Point3d(vector.getX(), vector.getY(), z);
         assertTrue(cylinder.contains(point));
      }
   }
	@ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testOutideCylinderPoints()
   {
      Random random = new Random(98716L);
      
      CollisionCylinder cylinder = new CollisionCylinder(new RigidBodyTransform(), 5.0, 10.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = (random.nextBoolean()?1.0:-1.0) * (5.0 + 10.0 * random.nextDouble());
         Vector2d vector = new Vector2d(random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         double z = (random.nextBoolean()?1.0:-1.0) * (5.0 + 10.0 * random.nextDouble());  
         
         
         Point3d point = new Point3d(vector.getX(), vector.getY(), z);
         assertFalse(cylinder.contains(point));
      }
   }
   
   
}
