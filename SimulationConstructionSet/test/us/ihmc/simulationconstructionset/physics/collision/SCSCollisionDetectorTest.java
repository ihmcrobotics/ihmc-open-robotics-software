package us.ihmc.simulationconstructionset.physics.collision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.RigidBodyTransform;


/**
 * Tests compliance to the {@link us.ihmc.simulationconstructionset.physics.ScsCollisionDetector}
 *
 * @author Peter Abeles
 */
public abstract class SCSCollisionDetectorTest
{
   protected ScsCollisionDetector collisionDetector;

   public abstract ScsCollisionDetector createCollisionInterface();

   double margin;

   @Before
   public void init()
   {
      margin = 0.02;
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void createBox_checkBounds_noHit()
   {
      collisionDetector = createCollisionInterface();

      collisionDetector.initialize(new CheckNoCollision());

      Link linkA = cube("A", 10, 0.5, 1, 1.5);
      Link linkB = cube("B", 10, 0.75, 1.2, 1.7);

      double a[] = new double[] {0.5 + 0.75, 1 + 1.2, 1.5 + 1.7};

      // add a bit of separation to ensure they don't collide
      double tau = 0.001;

      // should just barely not intersect
      for (int i = 0; i < 3; i++)
      {
         double Tx, Ty, Tz;
         Tx = Ty = Tz = 0;
         if (i == 0)
            Tx = a[i] / 2 + tau;
         if (i == 1)
            Ty = a[i] / 2 + tau;
         if (i == 2)
            Tz = a[i] / 2 + tau;

         linkA.getParentJoint()._setTransformToWorld(translate(Tx, Ty, Tz));
         linkB.getParentJoint()._setTransformToWorld(translate(-Tx, -Ty, -Tz));

         collisionDetector.performCollisionDetection();
      }
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void createBox_checkBounds_hit()
   {
      collisionDetector = createCollisionInterface();

      double tau = 0.001;

      collisionDetector.initialize(new CheckCollision(tau * 2));

      Link linkA = cube("A", 10, 0.5, 1, 1.5);
      Link linkB = cube("B", 10, 0.75, 1.2, 1.7);

      double a[] = new double[] {0.5 + 0.75, 1 + 1.2, 1.5 + 1.7};

      // should just barely interesect
      for (int i = 0; i < 3; i++)
      {
         double Tx, Ty, Tz;
         Tx = Ty = Tz = 0;
         if (i == 0)
            Tx = a[i] / 2 - tau;
         if (i == 1)
            Ty = a[i] / 2 - tau;
         if (i == 2)
            Tz = a[i] / 2 - tau;

         linkA.getParentJoint()._setTransformToWorld(translate(Tx, Ty, Tz));
         linkB.getParentJoint()._setTransformToWorld(translate(-Tx, -Ty, -Tz));

         collisionDetector.performCollisionDetection();
      }
   }

   /**
    * Make a small object and see if it detects the collision correctly.  Small objects aren't already handled correctly
    */

	@EstimatedDuration
	@Test(timeout=300000)
   public void createBox_Collision_Small()
   {
      double tau = 0.0001;
      double r = 0.005;    // box width = 1cm

//    margin = tau/10;

      collisionDetector = createCollisionInterface();
      collisionDetector.initialize(new CheckCollision(tau * 2));

      Link linkA = cube("A", 10, r, r, r);
      Link linkB = cube("B", 10, r, r, r);

      // barely intersect
      linkA.getParentJoint()._setTransformToWorld(translate(2 * r - tau, 0, 0));
      collisionDetector.performCollisionDetection();

      // barely miss
      collisionDetector = createCollisionInterface();
      collisionDetector.initialize(new CheckNoCollision());

      linkA = cube("A", 10, r, r, r);
      linkB = cube("B", 10, r, r, r);

      linkA.getParentJoint()._setTransformToWorld(translate(2 * r + tau, 0, 0));
      collisionDetector.performCollisionDetection();
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void collisionMask_hit()
   {
      collisionDetector = createCollisionInterface();

      CheckCollisionMasks check = new CheckCollisionMasks();

      collisionDetector.initialize(check);

      // all 3 shapes will overlap.  the mask determines what intersects
      Link linkA = cube("A", 10, null, 0.5, 1, 1.5, 0x01, 0x02);
      Link linkB = cube("A", 10, null, 0.75, 1.2, 1.7, 0x02, 0x01);
      Link linkC = cube("A", 10, null, 10, 10, 10, 0x04, 0x04);

      // just do an offset so that not everything is at the origin
      linkB.getParentJoint()._setTransformToWorld(translate(0.4, 0, 0));

      collisionDetector.performCollisionDetection();

      assertEquals(1, check.totalCollisions);
   }

   /**
    * Makes sure the offset from the link is handled correctly
    */

	@EstimatedDuration
	@Test(timeout=300000)
   public void checkCollisionShape_offset()
   {
      collisionDetector = createCollisionInterface();

      CheckCollisionMasks check = new CheckCollisionMasks();

      collisionDetector.initialize(check);

      RigidBodyTransform offset = new RigidBodyTransform();
      offset.setTranslation(new Vector3d(0, 0, -1.7));

      Link linkA = cube("A", 10, null, 1, 1, 1, 2, 2);
      Link linkB = cube("B", 10, null, 1, 1, 1, 2, 2);
      Link linkC = cube("C", 10, offset, 1, 1, 1, 2, 2);

      linkA.getParentJoint()._setTransformToWorld(translate(0, 0, 0.5));
      linkB.getParentJoint()._setTransformToWorld(translate(0, 0, 0.5));

      collisionDetector.performCollisionDetection();

      // only A and B should collide
      assertEquals(1, check.totalCollisions);
   }

   public Link cube(String name, double mass, double radiusX, double radiusY, double radiusZ)
   {
      return cube(name, mass, null, radiusX, radiusY, radiusZ, 0xFFFFFFFF, 0xFFFFFFFF);
   }

   public Link cube(String name, double mass, RigidBodyTransform shapeToLink, double radiusX, double radiusY, double radiusZ, int collisionGroup, int collisionMask)
   {
      Link ret = new Link(name);

      ret._setParentJoint(new DummyJoint());

//    ret.setMass(mass);
//    ret.setMomentOfInertia(0.1 * mass, 0.1 * mass, 0.1 * mass);
//    ret.enableCollisions(10,null);

      CollisionShapeFactory factory = collisionDetector.getShapeFactory();
      factory.setMargin(0.02);
      CollisionShapeDescription shapeDesc = factory.createBox(radiusX, radiusY, radiusZ);
      factory.addShape(ret, shapeToLink, shapeDesc, false,  collisionGroup, collisionMask);

      return ret;
   }

   private RigidBodyTransform translate(double x, double y, double z)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      ret.setTranslationAndIdentityRotation(new Vector3d(x, y, z));

      return ret;
   }

   private static class CheckCollision implements CollisionHandler
   {
      double tolerance;

      private CheckCollision(double tolerance)
      {
         this.tolerance = tolerance;
      }

      public void initialize(ScsCollisionDetector collision)
      {
      }

      public void maintenanceBeforeCollisionDetection()
      {
      }
      
      public void maintenanceAfterCollisionDetection()
      {
      }

      public void addListener(Listener listener)
      {
      }

      public void handle(CollisionShape shapeA, CollisionShape shapeB, Contacts contacts)
      {
         assertTrue(contacts.getNumContacts() > 0);

         // there should be no collision, so distance > 0 or no contacts
         for (int i = 0; i < contacts.getNumContacts(); i++)
         {
            assertTrue(contacts.getDistance(i) <= tolerance);
         }
      }
   }


   private static class CheckNoCollision implements CollisionHandler
   {
      public void initialize(ScsCollisionDetector collision)
      {
      }

      public void maintenanceBeforeCollisionDetection()
      {
      }
      
      public void maintenanceAfterCollisionDetection()
      {
      }
      
      public void addListener(Listener listener)
      {
      }

      public void handle(CollisionShape shapeA, CollisionShape shapeB, Contacts contacts)
      {
         // there should be no collision, so distance > 0 or no contacts
         for (int i = 0; i < contacts.getNumContacts(); i++)
         {
            double d = contacts.getDistance(i);
            assertTrue("distance = " + d, d > -1e-8);
         }
      }
   }


   private static class CheckCollisionMasks implements CollisionHandler
   {
      int totalCollisions = 0;

      public void initialize(ScsCollisionDetector collision)
      {
      }

      public void maintenanceBeforeCollisionDetection()
      {
      }
      
      public void maintenanceAfterCollisionDetection()
      {
      }

      public void addListener(Listener listener)
      {
      }

      public void handle(CollisionShape shapeA, CollisionShape shapeB, Contacts contacts)
      {
         totalCollisions++;

         assertTrue((shapeA.getCollisionMask() & shapeB.getGroupMask()) != 0 || (shapeB.getCollisionMask() & shapeA.getGroupMask()) != 0);
      }
   }


   private static class DoNothingCollision implements CollisionHandler
   {
      public void initialize(ScsCollisionDetector collision)
      {
      }

      public void maintenanceBeforeCollisionDetection()
      {
      }
      
      public void maintenanceAfterCollisionDetection()
      {
      }

      public void addListener(Listener listener)
      {
      }

      public void handle(CollisionShape shapeA, CollisionShape shapeB, Contacts contacts)
      {
      }
   }


   private static class DummyJoint extends Joint
   {
      protected DummyJoint()
      {
         super("Stupid", new Vector3d(), null, 1);
      }

      @Override
      protected void update()
      {
      }

   }

}
