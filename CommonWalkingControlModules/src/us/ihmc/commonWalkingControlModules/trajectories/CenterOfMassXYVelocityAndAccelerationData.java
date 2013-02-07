package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class CenterOfMassXYVelocityAndAccelerationData
{
   private Point2d comPosition = new Point2d();
   private Vector2d comVelocity = new Vector2d();
   private Vector2d comAcceleration = new Vector2d();
   
   public void set(CenterOfMassXYVelocityAndAccelerationData comData)
   {
      this.comPosition.set(comData.comPosition);
      this.comVelocity.set(comData.comVelocity);
      this.comAcceleration.set(comData.comAcceleration);
   }
   
   public void getCoMPosition(Point2d comPositionToPack)
   {
      comPositionToPack.set(this.comPosition);
   }
   
   public void getCoMVelocity(Vector2d comVelocityToPack)
   {
      comVelocityToPack.set(this.comVelocity);
   }
   
   public void getCoMAcceleration(Vector2d comAccelerationToPack)
   {
      comAccelerationToPack.set(this.comAcceleration);
   }
}
