package us.ihmc.rdx.input;

public class ImGui3DViewPickResult
{
   private double distanceToCamera = Double.POSITIVE_INFINITY;

   public void reset()
   {
      distanceToCamera = Double.POSITIVE_INFINITY;
   }

   public void addPickCollision(double distanceToCamera)
   {
      if (distanceToCamera < this.distanceToCamera)
      {
         this.distanceToCamera = distanceToCamera;
      }
   }

   public boolean getPickCollisionWasAddedSinceReset()
   {
      return distanceToCamera < Double.POSITIVE_INFINITY;
   }

   public void setDistanceToCamera(double distanceToCamera)
   {
      this.distanceToCamera = distanceToCamera;
   }

   public double getDistanceToCamera()
   {
      return distanceToCamera;
   }
}
