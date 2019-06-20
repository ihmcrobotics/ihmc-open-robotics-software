package us.ihmc.footstepPlanning.graphSearch.collision;

public class BodyCollisionData
{
   /**
    * Whether bounding box collision was detected
    */
   private boolean collisionDetected = false;

   /**
    * Distance of closest detected point to bounding box if no collision was detected
    */
   private double distanceFromBoundingBox = Double.NaN;

   /**
    * Distance of closest detected points along the x-axis if no collision is detected. Used to translate footstep away from near collisions
    */
   private double distanceOfClosestPointInFront = Double.NaN;
   private double distanceOfClosestPointInBack = Double.NaN;

   public boolean isCollisionDetected()
   {
      return collisionDetected;
   }

   public void setDistanceFromBoundingBox(double distanceFromBoundingBox)
   {
      this.distanceFromBoundingBox = distanceFromBoundingBox;
   }

   public void setCollisionDetected(boolean collisionDetected)
   {
      this.collisionDetected = collisionDetected;
   }

   public double getDistanceFromBoundingBox()
   {
      return distanceFromBoundingBox;
   }

   public double getDistanceOfClosestPointInFront()
   {
      return distanceOfClosestPointInFront;
   }

   public double getDistanceOfClosestPointInBack()
   {
      return distanceOfClosestPointInBack;
   }

   public void setDistanceOfClosestPointInFront(double distanceOfClosestPointInFront)
   {
      this.distanceOfClosestPointInFront = distanceOfClosestPointInFront;
   }

   public void setDistanceOfClosestPointInBack(double distanceOfClosestPointInBack)
   {
      this.distanceOfClosestPointInBack = distanceOfClosestPointInBack;
   }
}
