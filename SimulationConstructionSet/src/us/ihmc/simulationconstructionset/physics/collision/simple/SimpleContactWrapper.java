package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.Contacts;

public class SimpleContactWrapper implements Contacts
{
   private CollisionShape shapeA;
   private CollisionShape shapeB;

   private final ArrayList<SingleContact> contacts = new ArrayList<>();

   public SimpleContactWrapper(CollisionShape shapeA, CollisionShape shapeB)
   {
      this.shapeA = shapeA;
      this.shapeB = shapeB;
   }

   @Override
   public void set(Contacts collision)
   {
      clear();

      this.shapeA = collision.getShapeA();
      this.shapeB = collision.getShapeB();

      addAll(collision);
   }

   public void clear()
   {
      contacts.clear();
   }

   @Override
   public void addAll(Contacts contactsToAdd)
   {
      CollisionShape shapeA = contactsToAdd.getShapeA();
      CollisionShape shapeB = contactsToAdd.getShapeB();

      boolean switched = (shapeA == this.shapeB);

      if (switched)
      {
         if (this.shapeA != shapeB)
         {
            throw new RuntimeException();
         }
      }
      else
      {
         if ((this.shapeA != shapeA) || (this.shapeB != shapeB))
         {
            throw new RuntimeException();
         }
      }

      int numberOfContacts = contactsToAdd.getNumberOfContacts();

      for (int i = 0; i < numberOfContacts; i++)
      {
         SingleContact singleContact = new SingleContact();

         Point3D worldAPoint = new Point3D();
         contactsToAdd.getWorldA(i, worldAPoint);

         Point3D worldBPoint = new Point3D();
         contactsToAdd.getWorldB(i, worldBPoint);

         Vector3D normalAVector = new Vector3D();

         contactsToAdd.getWorldNormal(i, normalAVector);

         double distance = contactsToAdd.getDistance(i);

         if (switched)
         {
            normalAVector.scale(-1.0);
            singleContact.set(worldBPoint, worldAPoint, normalAVector, distance);
         }
         else
         {
            singleContact.set(worldAPoint, worldBPoint, normalAVector, distance);
         }

         contacts.add(singleContact);
      }

   }
   
   
   public void addAllReplaceNearby(Contacts contactsToAdd)
   {  
      CollisionShape shapeA = contactsToAdd.getShapeA();
      CollisionShape shapeB = contactsToAdd.getShapeB();

      boolean switched = (shapeA == this.shapeB);

      if (switched)
      {
         if (this.shapeA != shapeB)
         {
            throw new RuntimeException();
         }
      }
      else
      {
         if ((this.shapeA != shapeA) || (this.shapeB != shapeB))
         {
            throw new RuntimeException();
         }
      }

      int numberOfContacts = contactsToAdd.getNumberOfContacts();

      for (int i = 0; i < numberOfContacts; i++)
      {
         // Find nearby one:
         SingleContact singleContact = null;

         for (int j=0; j<contacts.size(); j++)
         {
            SingleContact singleContactToCheck = contacts.get(j);
            
            Point3D worldAToCheck = new Point3D();
            singleContactToCheck.getWorldA(worldAToCheck);
            
            Point3D worldAPoint = new Point3D();
            if (switched)
            {
               contactsToAdd.getWorldB(i, worldAPoint);
            }
            else
            {
               contactsToAdd.getWorldA(i, worldAPoint);
            }

            if (worldAToCheck.distance(worldAPoint) < 0.003)
            {
               singleContact = singleContactToCheck;
            }
         }
         
         if (singleContact == null)
         {
            singleContact = new SingleContact();
            contacts.add(singleContact);
         }

         Point3D worldAPoint = new Point3D();
         contactsToAdd.getWorldA(i, worldAPoint);

         Point3D worldBPoint = new Point3D();
         contactsToAdd.getWorldB(i, worldBPoint);

         Vector3D normalAVector = new Vector3D();

         contactsToAdd.getWorldNormal(i, normalAVector);

         double distance = contactsToAdd.getDistance(i);

         if (switched)
         {
            normalAVector.scale(-1.0);
            singleContact.set(worldBPoint, worldAPoint, normalAVector, distance);
         }
         else
         {
            singleContact.set(worldAPoint, worldBPoint, normalAVector, distance);
         }

      }

   }


   public void addContact(Point3D pointA, Point3D pointB, Vector3D normalA, double distance)
   {
      SingleContact singleContact = new SingleContact();
      singleContact.set(pointA, pointB, normalA, distance);
      this.contacts.add(singleContact);
   }

   @Override
   public int getNumberOfContacts()
   {
      return contacts.size();
   }

   @Override
   public void getWorldA(int which, Point3D location)
   {
      SingleContact singleContact = contacts.get(which);
      singleContact.getWorldA(location);
   }

   @Override
   public void getWorldB(int which, Point3D location)
   {
      SingleContact singleContact = contacts.get(which);
      singleContact.getWorldB(location);
   }

   @Override
   public double getDistance(int which)
   {
      SingleContact singleContact = contacts.get(which);
      return singleContact.getDistance();
   }

   @Override
   public void getWorldNormal(int which, Vector3D normalToPack)
   {
      SingleContact singleContact = contacts.get(which);
      singleContact.getNormalA(normalToPack);
   }

   @Override
   public boolean isNormalOnA()
   {
      return true;
   }

   @Override
   public CollisionShape getShapeA()
   {
      return shapeA;
   }

   @Override
   public CollisionShape getShapeB()
   {
      return shapeB;
   }

}
