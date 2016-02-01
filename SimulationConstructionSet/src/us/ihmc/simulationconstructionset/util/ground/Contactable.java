package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.GroundContactPoint;

public interface Contactable
{   
   public abstract boolean isClose(Point3d pointInWorldToCheck);
   public abstract boolean isPointOnOrInside(Point3d pointInWorldToCheck);
   public abstract void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck);
//   public abstract boolean checkIfInside(Point3d pointToCheck, Point3d intersectionToPack, Vector3d normalToPack);
   
   public abstract void updateContactPoints();
   public abstract int getAndLockAvailableContactPoint();
   public abstract void unlockContactPoint(GroundContactPoint groundContactPoint);
   public abstract GroundContactPoint getLockedContactPoint(int contactPointIndex);
   
}
