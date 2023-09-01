package us.ihmc.rdx.vr;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * A pick result is intended to be created as a field and reused.
 * Call reset(), add collisions if they are present, and add this
 * pick result to the VR context.
 */
public class RDXVRPickResult
{
   private double distanceToControllerPickPoint = Double.POSITIVE_INFINITY;
   private final Point3D closestPointOnSurface = new Point3D();
   private Object objectBeingPicked = null;
   private String pickedObjectName = "";

   public void setPointingAtCollision(double distanceToControllerPickPoint)
   {
      this.distanceToControllerPickPoint = distanceToControllerPickPoint;
   }

   public void setHoveringCollsion(Point3DReadOnly controllerPickPosition, Point3DReadOnly closestPointOnSurface)
   {
      this.closestPointOnSurface.set(closestPointOnSurface);
      distanceToControllerPickPoint = -controllerPickPosition.distance(this.closestPointOnSurface);
   }

   public boolean isHoveringNotPointingAt()
   {
      return distanceToControllerPickPoint <= 0.0;
   }

   public double getDistanceToControllerPickPoint()
   {
      return distanceToControllerPickPoint;
   }

   public Object getObjectBeingPicked()
   {
      return objectBeingPicked;
   }

   public void setPickedObjectID(Object objectBeingPicked, String pickedObjectName)
   {
      this.pickedObjectName = pickedObjectName;
   }

   public String getPickedObjectName()
   {
      return pickedObjectName;
   }

   public Point3D getClosestPointOnSurface()
   {
      return closestPointOnSurface;
   }
}
