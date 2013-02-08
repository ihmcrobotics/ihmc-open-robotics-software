package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class CenterOfMassXYVelocityAndAccelerationData
{
   private Point2d comXYPosition = new Point2d();
   private Vector2d comXYVelocity = new Vector2d();
   private Vector2d comXYAcceleration = new Vector2d();
   
   public void set(CenterOfMassXYVelocityAndAccelerationData comXYData)
   {
      this.comXYPosition.set(comXYData.comXYPosition);
      this.comXYVelocity.set(comXYData.comXYVelocity);
      this.comXYAcceleration.set(comXYData.comXYAcceleration);
   }
   
   public void getCoMXYPosition(Point2d comXYPositionToPack)
   {
      comXYPositionToPack.set(this.comXYPosition);
   }
   
   public void getCoMXYVelocity(Vector2d comXYVelocityToPack)
   {
      comXYVelocityToPack.set(this.comXYVelocity);
   }
   
   public void getCoMXYAcceleration(Vector2d comXYAccelerationToPack)
   {
      comXYAccelerationToPack.set(this.comXYAcceleration);
   }
   
   public void setCoMXYPosition(Point2d comXYPosition)
   {
      this.comXYPosition.set(comXYPosition);
   }
   
   public void setCoMXYVelocity(Vector2d comXYVelocity)
   {
      this.comXYVelocity.set(comXYVelocity);
   }
   
   public void setCoMXYAcceleration(Vector2d comXYAcceleration)
   {
      this.comXYVelocity.set(comXYAcceleration);
   }
}
