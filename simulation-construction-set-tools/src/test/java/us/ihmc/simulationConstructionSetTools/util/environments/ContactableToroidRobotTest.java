package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableToroidRobot;

import static org.junit.jupiter.api.Assertions.*;

public class ContactableToroidRobotTest
{

	@Test
   public void testIsPointOnOrInsideAtOrigin()
   {
      double majorRadius = ContactableToroidRobot.DEFAULT_RADIUS;
      double minorRadius = ContactableToroidRobot.DEFAULT_THICKNESS;
      double delta = 5e-4;

      ContactableToroidRobot bot = new ContactableToroidRobot("bot", new RigidBodyTransform());

      Point3D testPoint = new Point3D();

      for (double x = majorRadius - minorRadius; x < majorRadius + minorRadius; x += delta)
      {
         for (double y = majorRadius - minorRadius; y < majorRadius + minorRadius; y += delta)
         {
            for (double z = -minorRadius; z < minorRadius; z += delta)
            {
               double x_sq = x * x;
               double y_sq = y * y;
               double z_sq = z * z;
               double rma_sq = (majorRadius + minorRadius) * (majorRadius + minorRadius);
               double rmi_sq = (minorRadius * minorRadius);               

               if ((x_sq + y_sq < rma_sq) &&  ((x_sq + y_sq) + z_sq < rmi_sq))
               {
                  testPoint.set(x, y, z);
                  assertTrue(bot.isPointOnOrInside(testPoint), "Nope: " + testPoint);
               }
            }
         }
      }
   }

	@Test
   public void testIsPointOnOrInsideNotAtOriginUsingTransform()
   {
      Random random = new Random(1972L);
      
      double majorRadius = ContactableToroidRobot.DEFAULT_RADIUS;
      double minorRadius = ContactableToroidRobot.DEFAULT_THICKNESS;
      double delta = 5e-4;
      
      Vector3D randomVector = EuclidCoreRandomTools.nextVector3D(random);
      RigidBodyTransform transform3d = new RigidBodyTransform();
      transform3d.getTranslation().set(randomVector);

      ContactableToroidRobot bot = new ContactableToroidRobot("bot", transform3d);

      Point3D pt = new Point3D();
      
      for (double x = randomVector.getX() + majorRadius - minorRadius; x < randomVector.getX() + majorRadius + minorRadius; x += delta)
      {
         for (double y = randomVector.getY() + majorRadius - minorRadius; y < randomVector.getY() + majorRadius + minorRadius; y += delta)
         {
            for (double z = randomVector.getZ() - minorRadius; z < randomVector.getZ() + minorRadius; z += delta)
            {
               double x_sq = x * x;
               double y_sq = y * y;
               double z_sq = z * z;
               double rma_sq = (randomVector.getX() + majorRadius + minorRadius) * (randomVector.getX() + majorRadius + minorRadius);
               double rmi_sq = (minorRadius * minorRadius);               

               if ((x_sq + y_sq < rma_sq) &&  ((x_sq + y_sq) + z_sq < rmi_sq))
               {
                  pt.set(x, y, z);
                  assertTrue(bot.isPointOnOrInside(pt), "Nope: " + pt);
               }
            }
         }
      }
   }

	@Test
   public void testPointIsntInsideWhenUsingComOffset()
   {
      Random random = new Random(1972L);
      
      double majorRadius = ContactableToroidRobot.DEFAULT_RADIUS;
      double minorRadius = ContactableToroidRobot.DEFAULT_THICKNESS;
      double delta = 5e-4;
      
      Vector3D vector3d = EuclidCoreRandomTools.nextVector3D(random);
      RigidBodyTransform randomTransform = new RigidBodyTransform();
      randomTransform.getTranslation().set(vector3d);

      ContactableToroidRobot bot = new ContactableToroidRobot("bot", randomTransform);

      Point3D pt = new Point3D();
      
      for (double x = vector3d.getX() + majorRadius - minorRadius; x < vector3d.getX() + majorRadius + minorRadius; x += delta)
      {
         for (double y = vector3d.getY() + majorRadius - minorRadius; y < vector3d.getY() + majorRadius + minorRadius; y += delta)
         {
            for (double z = vector3d.getZ() - minorRadius; z < vector3d.getZ() + minorRadius; z += delta)
            {
               double x_sq = x * x;
               double y_sq = y * y;
               double z_sq = z * z;
               double rma_sq = (vector3d.getX() + majorRadius + minorRadius) * (vector3d.getX() + majorRadius + minorRadius);
               double rmi_sq = (minorRadius * minorRadius);               

               if ((x_sq + y_sq < rma_sq) &&  ((x_sq + y_sq) + z_sq < rmi_sq))
               {
                  pt.set(x, y, z);
                  assertFalse(bot.isPointOnOrInside(pt), "Nope: " + pt);
               }
            }
         }
      }
   }

}
