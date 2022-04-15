package us.ihmc.gdx.input;

public class ImGui3DViewPickResult
{
   private boolean pickIntersects = false;
   private double distanceToCamera = Double.POSITIVE_INFINITY;

   public void reset()
   {
      pickIntersects = false;
      distanceToCamera = Double.POSITIVE_INFINITY;
   }

   public void addPickCollision(double distanceToCamera)
   {
      pickIntersects = true;
      if (distanceToCamera < this.distanceToCamera)
      {
         this.distanceToCamera = distanceToCamera;
      }
   }

   public void setPickIntersects(boolean pickIntersects)
   {
      this.pickIntersects = pickIntersects;
   }

   public boolean getPickIntersects()
   {
      return pickIntersects;
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
