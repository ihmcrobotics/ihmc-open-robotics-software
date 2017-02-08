package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.Contacts;

public class SimpleContactWrapper implements Contacts
{
   private CollisionShape shapeA;
   private CollisionShape shapeB;

   // TODO: Make a contact point class and have an array of them, rather than this silliness:
   private final ArrayList<Point3d> worldA = new ArrayList<Point3d>();
   private final ArrayList<Point3d> worldB = new ArrayList<Point3d>();
   private final ArrayList<Vector3d> normalA = new ArrayList<Vector3d>();
   private final ArrayList<Double> distances = new ArrayList<Double>();

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
      worldA.clear();
      worldB.clear();
      normalA.clear();
      distances.clear();
   }

   @Override
   public void addAll(Contacts contacts)
   {
      CollisionShape shapeA = contacts.getShapeA();
      CollisionShape shapeB = contacts.getShapeB();

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

      int numberOfContacts = contacts.getNumberOfContacts();

      for (int i = 0; i < numberOfContacts; i++)
      {
         Point3d worldAPoint = new Point3d();
         contacts.getWorldA(i, worldAPoint);
         worldA.add(worldAPoint);

         Point3d worldBPoint = new Point3d();
         contacts.getWorldB(i, worldBPoint);
         worldB.add(worldBPoint);

         Vector3d normalAVector = new Vector3d();

         contacts.getWorldNormal(i, normalAVector);
         if (switched)
         {
            normalAVector.scale(-1.0);
         }
         normalA.add(normalAVector);

         distances.add(contacts.getDistance(i));
      }

   }

   public void addContact(Point3d pointA, Point3d pointB, Vector3d normalA, double distance)
   {
      this.worldA.add(pointA);
      this.worldB.add(pointB);
      this.normalA.add(normalA);
      this.distances.add(distance);
   }

   @Override
   public int getNumberOfContacts()
   {
      return worldA.size();
   }

   @Override
   public void getWorldA(int which, Point3d location)
   {
      Point3d point3d = worldA.get(which);
      location.set(point3d);
   }

   @Override
   public void getWorldB(int which, Point3d location)
   {
      Point3d point3d = worldB.get(which);
      location.set(point3d);
   }

   @Override
   public double getDistance(int which)
   {
      return distances.get(which);
   }

   @Override
   public void getWorldNormal(int which, Vector3d normalToPack)
   {
      normalToPack.set(normalA.get(which));
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
