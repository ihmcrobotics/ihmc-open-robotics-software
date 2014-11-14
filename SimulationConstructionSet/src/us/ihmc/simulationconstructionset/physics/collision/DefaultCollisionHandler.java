package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;

/**
 * @author Peter Abeles
 */
public class DefaultCollisionHandler implements CollisionHandler
{
   private Vector3d normal, negative_normal = new Vector3d();

   private Point3d point1 = new Point3d();
   private Point3d point2 = new Point3d();

   private List<Listener> listeners = new ArrayList<Listener>();

   // coefficent of restitution.
   private final double epsilon;

   // coefficient of friction
   private final double mu;

   /**
    *
    * @param epsilon coefficent of restitution.
    * @param mu coefficient of friction
    * @param robot Robot model
    */
   public DefaultCollisionHandler(double epsilon, double mu)
   {
      this.epsilon = epsilon;
      this.mu = mu;
   }

   public void initialize(ScsCollisionDetector collision)
   {
   }

   public void maintenanceBeforeCollisionDetection()
   {
      shapesInContactList.clear();
   }
   
   public void maintenanceAfterCollisionDetection()
   {
      Collections.shuffle(shapesInContactList);
      
      for (int i=0; i<shapesInContactList.size(); i++)
      {
         ShapesInContact shapesInContact = shapesInContactList.get(i);
         
         CollisionShape shape1 = shapesInContact.getShape1();
         CollisionShape shape2 = shapesInContact.getShape2();
         Contacts contacts = shapesInContact.getContacts();
         handleLocal(shape1, shape2, contacts);
      }
   }

   
   private final ArrayList<ShapesInContact> shapesInContactList = new ArrayList<DefaultCollisionHandler.ShapesInContact>();
   
   public void handle(CollisionShape shape1, CollisionShape shape2, Contacts contacts)
   {
      shapesInContactList.add(new ShapesInContact(shape1, shape2, contacts));
//      handleLocal(shape1, shape2, contacts);
   }
   
   private final ArrayList<Integer> indices = new ArrayList<Integer>();

   private void handleLocal(CollisionShape shape1, CollisionShape shape2, Contacts contacts)
   {
      boolean shapeOneIsGround = shape1.isGround();
      boolean shapeTwoIsGround = shape2.isGround();
      if (shapeOneIsGround && shapeTwoIsGround) throw new RuntimeException("Both shapes are ground. Shouldn't be contacting!!");
      
      int numberOfContacts = contacts.getNumContacts();
      indices.clear();
      
      for (int i=0; i<numberOfContacts; i++)
      {
         indices.add(i);
      }
      
      // TODO: Sims won't sim same way twice, but I don't think they do anyway...
      Collections.shuffle(indices);
      
      for (int j = 0; j < numberOfContacts; j++)
      {
         int i = indices.get(j);
         double distance = contacts.getDistance(i);

         if (distance > 0)
            continue;

         contacts.getWorldA(i, point1);
         contacts.getWorldB(i, point2);

         normal = contacts.getWorldNormal(i);

         if (!contacts.isNormalOnA())
         {
            normal.scale(-1.0);
         }

         // TODO handle the case where the object is embedded inside the object and the normal is invalid
         if (Double.isNaN(normal.x))
            throw new RuntimeException("Normal is invalid");

         negative_normal.set(normal);
         negative_normal.scale(-1.0);
         
         ExternalForcePoint externalForcePointOne = shape1.getLink().ef_collision;
         ExternalForcePoint externalForcePointTwo = shape2.getLink().ef_collision;

         // +++JEP: For now. Make more efficient later. Don't need an ef_point really...
         externalForcePointOne.setOffsetWorld(point1.x, point1.y, point1.z);    // Put the external force points in the right places.
         externalForcePointTwo.setOffsetWorld(point2.x, point2.y, point2.z);

         // Update the robot and its velocity:
         Robot robot1 = shape1.getLink().getParentJoint().getRobot();
         Robot robot2 = shape2.getLink().getParentJoint().getRobot();

         robot1.updateVelocities();
         robot1.update();

         if (robot2 != robot1)
         {
            robot2.updateVelocities();
            robot2.update();
         }
         
         // Resolve the collision:
         Vector3d p_world = new Vector3d();

         // +++JEP: epsilon, mu hardcoded on construction right now. Need to change that!
         
         if (shapeTwoIsGround)
         {
            Vector3d velocityWorld = new Vector3d(0.0, 0.0, 0.0);
            externalForcePointOne.resolveCollision(velocityWorld , negative_normal, epsilon, mu, p_world);    // link1.epsilon, link1.mu, p_world);
         }
         else if (shapeOneIsGround)
         {
            Vector3d velocityWorld = new Vector3d(0.0, 0.0, 0.0);
            externalForcePointTwo.resolveCollision(velocityWorld, normal, epsilon, mu, p_world);    // link1.epsilon, link1.mu, p_world);
         }
         else
         {
            externalForcePointOne.resolveCollision(externalForcePointTwo, negative_normal, epsilon, mu, p_world);    // link1.epsilon, link1.mu, p_world);
         }
         
         for (Listener l : listeners)
         {
            l.collision(shape1, shape2, externalForcePointOne, externalForcePointTwo, null, null);
         }

      }
   }

   public void addListener(Listener listener)
   {
      listeners.add(listener);
   }
   
   private class ShapesInContact
   {
      private final CollisionShape shape1, shape2;
      private final Contacts contacts;
      
      public ShapesInContact(CollisionShape shape1, CollisionShape shape2, Contacts contacts)
      {
         this.shape1 = shape1;
         this.shape2 = shape2;
         this.contacts = contacts;
      }
      
      public CollisionShape getShape1()
      {
         return shape1;
      }
      
      public CollisionShape getShape2()
      {
         return shape2;
      }
      
      public Contacts getContacts()
      {
         return contacts;
      }
   }
}
