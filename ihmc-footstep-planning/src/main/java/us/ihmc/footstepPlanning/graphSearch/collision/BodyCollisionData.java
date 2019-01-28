package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

class BodyCollisionData
{
   private final Point2D collisionPointInBodyFrame = new Point2D();
   private boolean collisionDetected = false;

   public BodyCollisionData()
   {
      collisionPointInBodyFrame.setToNaN();
   }

   public void setCollisionDetected(boolean collisionDetected)
   {
      this.collisionDetected = collisionDetected;
   }

   public void setCollisionPointInBodyFrame(double x, double y)
   {
      this.collisionPointInBodyFrame.set(x, y);
   }

   public boolean isCollisionDetected()
   {
      return collisionDetected;
   }

   public Point2DReadOnly getCollisionPointInBodyFrame()
   {
      return collisionPointInBodyFrame;
   }
}
