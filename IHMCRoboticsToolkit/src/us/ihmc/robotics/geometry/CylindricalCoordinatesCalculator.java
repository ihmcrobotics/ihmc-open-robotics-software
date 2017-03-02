package us.ihmc.robotics.geometry;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * @author twan
 *         Date: 5/21/13
 */
public class CylindricalCoordinatesCalculator
{

   private FramePoint position = new FramePoint();
   private final RotationMatrix preRotation = new RotationMatrix();
   private final RotationMatrix rotation = new RotationMatrix();
   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

   public FramePose getPoseFromCylindricalCoordinates(RobotSide robotSide, ReferenceFrame frame, double radiansFromYAxis, double radius, double z,
                                                              double outwardRotation, double pitchRotation)
   {
      getPosition(position, frame, radiansFromYAxis, radius, z);


      preRotation.setYawPitchRoll(0.0, Math.PI / 2.0, -Math.PI / 2.0);

      rotation.set(preRotation);
      rotation.appendRollRotation(robotSide.negateIfRightSide(Math.PI / 2.0) - radiansFromYAxis);
      rotation.appendYawRotation(robotSide.negateIfRightSide(outwardRotation));
      rotation.appendPitchRotation(pitchRotation);

      orientation.setIncludingFrame(frame, rotation);

      return new FramePose(position, orientation);
   }

   public static void getPosition(FramePoint pointToPack, ReferenceFrame frame, double angle, double radius, double z)
   {
      double x = radius * Math.cos(angle);
      double y = radius * Math.sin(angle);

      pointToPack.setIncludingFrame(frame, x, y, z);
   }

   public static void getVelocity(FrameVector velocityToPack, ReferenceFrame frame, double angle, double angleDot, double radius, double radiusDot, double zDot)
   {
      double cos = Math.cos(angle);
      double sin = Math.sin(angle);

      velocityToPack.setToZero(frame);

      double xDot = -radius * sin * angleDot + cos * radiusDot;
      double yDot =  radius * cos * angleDot + sin * radiusDot;

      velocityToPack.setX(xDot);
      velocityToPack.setY(yDot);
      velocityToPack.setZ(zDot);
   }

   public static void getAcceleration(FrameVector accelerationToPack, ReferenceFrame frame, double angle, double angleDot, double angleDDot, double radius, double radiusDot, double radiusDDot, double zDDot)
   {
      double cos = Math.cos(angle);
      double sin = Math.sin(angle);

      accelerationToPack.setToZero(frame);

      double xDDot = -cos * radius * MathTools.square(angleDot) - 2.0 * sin * angleDot * radiusDot - radius * sin * angleDDot + cos * radiusDDot;
      double yDDot = -sin * radius * MathTools.square(angleDot) + 2.0 * cos * angleDot * radiusDot + radius * cos * angleDDot + sin * radiusDDot;

      accelerationToPack.setX(xDDot);
      accelerationToPack.setY(yDDot);
      accelerationToPack.setZ(zDDot);
   }

   public static double getRadius(FramePoint position)
   {
      double x = position.getX();
      double y = position.getY();

      double radius = Math.hypot(x, y);
      return radius;
   }

   public static double getAngle(FramePoint position)
   {
      double x = position.getX();
      double y = position.getY();

      double angle = Math.atan2(y, x);
      return angle;
   }

   public static double getRadialVelocity(FramePoint position, FrameVector velocity)
   {
      position.checkReferenceFrameMatch(velocity);

      double x = position.getX();
      double y = position.getY();

      double xd = velocity.getX();
      double yd = velocity.getY();

      double radius = getRadius(position);
      return (x * xd + y * yd) / radius;
   }

   public static double getAngularVelocity(FramePoint position, FrameVector velocity)
   {
      position.checkReferenceFrameMatch(velocity);

      double x = position.getX();
      double y = position.getY();

      double xd = velocity.getX();
      double yd = velocity.getY();

      double radiusSquared = MathTools.square(x) + MathTools.square(y);
      return (x * yd - y * xd) / radiusSquared;
   }
}
