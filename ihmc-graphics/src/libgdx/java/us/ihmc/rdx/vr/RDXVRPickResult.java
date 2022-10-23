package us.ihmc.rdx.vr;

public class RDXVRPickResult
{
   private double distanceToControllerPickPoint = Double.POSITIVE_INFINITY;

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
}
