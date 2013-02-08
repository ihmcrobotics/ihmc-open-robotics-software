package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class CoMHeightTimeDerivativesCalculator
{   
   private final Point2d comXYPosition = new Point2d();
   private final Vector2d comXYVelocity = new Vector2d();
   private final Vector2d comXYAcceleration = new Vector2d();

   public void computeCoMHeightTimeDerivatives(CoMHeightData comHeightDataToPack, CenterOfMassXYVelocityAndAccelerationData xyVelocityAndAcceleration, CenterOfMassHeightPartialDerivativesData comPartialDerivatives)
   {
      xyVelocityAndAcceleration.getCoMXYPosition(comXYPosition);
      xyVelocityAndAcceleration.getCoMXYVelocity(comXYVelocity);
      xyVelocityAndAcceleration.getCoMXYAcceleration(comXYAcceleration);
      
      double comHeight = comPartialDerivatives.getCoMHeight();
      double dzDx = comPartialDerivatives.getPartialDzDx();
      double dzDy = comPartialDerivatives.getPartialDzDy();
      double d2zDx2 = comPartialDerivatives.getPartialD2zDx2();
      double d2zDy2 = comPartialDerivatives.getPartialD2zDy2();
      double d2zDxDy = comPartialDerivatives.getPartialD2zDxDy();
      
      double xDot = comXYVelocity.getX();
      double yDot = comXYVelocity.getX();
      
      double xDDot = comXYAcceleration.getX();
      double yDDot = comXYAcceleration.getX();
      
      double comHeightVelocity = dzDx * xDot + dzDy * yDot;
      double comHeightAcceleration = d2zDx2 * xDot * xDot + dzDx * xDDot+ d2zDy2 * yDot * yDot + dzDy * yDDot;
      
      comHeightDataToPack.setComHeight(comHeight);
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
   }
}
