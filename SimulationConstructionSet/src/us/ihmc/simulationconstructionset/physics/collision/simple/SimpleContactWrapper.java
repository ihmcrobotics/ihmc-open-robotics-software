package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.Contacts;

public class SimpleContactWrapper implements Contacts
{
   private final ArrayList<Point3d> worldA = new ArrayList<Point3d>();
   private final ArrayList<Point3d> worldB = new ArrayList<Point3d>();
   private final ArrayList<Vector3d> normalA = new ArrayList<Vector3d>();
   private final ArrayList<Double> distances = new ArrayList<Double>();

   public SimpleContactWrapper()
   {
   }

   public void addContact(Point3d pointA, Point3d pointB, Vector3d normalA, double distance)
   {
      this.worldA.add(pointA);
      this.worldB.add(pointB);
      this.normalA.add(normalA);
      this.distances.add(distance);
   }

   @Override
   public int getNumContacts()
   {
      return worldA.size();
   }

   @Override
   public Point3d getWorldA(int which, Point3d location)
   {
      Point3d point3d = worldA.get(which);
      location.set(point3d);
      return location;
   }

   @Override
   public Point3d getWorldB(int which, Point3d location)
   {
      Point3d point3d = worldB.get(which);
      location.set(point3d);
      return location;
   }

   @Override
   public double getDistance(int which)
   {
      return distances.get(which);
   }

   @Override
   public Vector3d getWorldNormal(int which)
   {
      return normalA.get(which);
   }

   @Override
   public boolean isNormalOnA()
   {
      return true;
   }

}
