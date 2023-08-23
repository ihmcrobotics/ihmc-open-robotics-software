package us.ihmc.rdx.vr;

/**
 * A pick result is intended to be created as a field and reused.
 * Call reset(), add collisions if they are present, and add this
 * pick result to the VR context.
 */
public class RDXVRPickResult
{
   private double distanceToControllerPickPoint = Double.POSITIVE_INFINITY;
   private Object objectBeingPicked = null;
   private String pickedObjectName = "";

   public void reset()
   {
      distanceToControllerPickPoint = Double.POSITIVE_INFINITY;
   }

   public void addPickCollision(double distanceToControllerPickPoint)
   {
      if (distanceToControllerPickPoint < this.distanceToControllerPickPoint)
      {
         this.distanceToControllerPickPoint = distanceToControllerPickPoint;
      }
   }

   public boolean getPickCollisionWasAddedSinceReset()
   {
      return distanceToControllerPickPoint < Double.POSITIVE_INFINITY;
   }

   public void setDistanceToControllerPickPoint(double distanceToControllerPickPoint)
   {
      this.distanceToControllerPickPoint = distanceToControllerPickPoint;
   }

   public double getDistanceToControllerPickPoint()
   {
      return distanceToControllerPickPoint;
   }

   /**
    * Optional. Used to filter access between various parts of the application.
    */
   public void setObjectBeingPicked(Object objectBeingPicked)
   {
      this.objectBeingPicked = objectBeingPicked;
   }

   public Object getObjectBeingPicked()
   {
      return objectBeingPicked;
   }

   public void setPickedObjectName(String pickedObjectName)
   {
      this.pickedObjectName = pickedObjectName;
   }

   public String getPickedObjectName()
   {
      return pickedObjectName;
   }
}
