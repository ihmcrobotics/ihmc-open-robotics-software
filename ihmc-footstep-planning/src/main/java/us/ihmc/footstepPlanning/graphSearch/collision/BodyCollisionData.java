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
}
