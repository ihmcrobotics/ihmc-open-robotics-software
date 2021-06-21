package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.robotics.geometry.PlanarRegion;

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

   private EuclidShape3DCollisionResult collisionResult = null;
   private final PlanarRegion planarRegion = new PlanarRegion();
   private final Box3D bodyBox = new Box3D();

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

   public EuclidShape3DCollisionResult getCollisionResult()
   {
      return collisionResult;
   }

   public void setCollisionResult(EuclidShape3DCollisionResult collisionResult)
   {
      this.collisionResult = collisionResult;
   }

   public PlanarRegion getPlanarRegion()
   {
      return planarRegion;
   }

   public Box3D getBodyBox()
   {
      return bodyBox;
   }
}
