package us.ihmc.simulationconstructionset.physics;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * A list of points between the two shapes which are in contact with each other.
 */
public interface Contacts
{
   /**
    * Total number of contacts found between the two shapes.
    */
   public int getNumContacts();

   /**
    * Location on shapeA that the contact occurred.  World coordinates.
    *
    * @param which Contact index.
    * @param location Storage for location.  If null a new instance will be declared.
    * @return Location of contact in world coordinates.
    */
   public Point3d getWorldA(int which, Point3d location);

   /**
    * Location on shapeB that the contact occurred.  World coordinates.
    *
    * @param which Contact index.
    * @param location Storage for location.  If null a new instance will be declared.
    * @return Location of contact in world coordinates.
    */
   public Point3d getWorldB(int which, Point3d location);

   /**
    * Distance between the two points.
    */
   public double getDistance(int which);

   /**
    * The normal between the collision in world coordintes.  The normal can be in reference to object A or B.  Call {@link #isNormalOnA()}  to
    * determin which one it is.
    *
    * @param which Contact index.
    * @return Normal in world coordinates.
    */
   public Vector3d getWorldNormal(int which);

   /**
    * Is the normal for the surface of A or B.
    *
    * @return true for normal being on A and false for B.
    */
   public boolean isNormalOnA();
}

