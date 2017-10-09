package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;

public interface Contactable
{   
   public abstract boolean isClose(Point3D pointInWorldToCheck);
   public abstract boolean isPointOnOrInside(Point3D pointInWorldToCheck);
   public abstract void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck);
//   public abstract boolean checkIfInside(Point3D pointToCheck, Point3D intersectionToPack, Vector3d normalToPack);
   
   public abstract void updateContactPoints();
   public abstract int getAndLockAvailableContactPoint();
   public abstract void unlockContactPoint(GroundContactPoint groundContactPoint);
   public abstract GroundContactPoint getLockedContactPoint(int contactPointIndex);
   
}
