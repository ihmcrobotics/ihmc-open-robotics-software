package us.ihmc.robotics.math;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;

public class DeadbandTools
{
   public static double applyDeadband(double deadbandSize, double value)
   {
      return applyDeadband(deadbandSize, 0.0, value);
   }

   public static double applyDeadband(double deadbandSize, double deadbandCenter, double value)
   {
      if (value > deadbandCenter)
         return Math.max(deadbandCenter, value - deadbandSize);
      else
         return Math.min(deadbandCenter, value + deadbandSize);
   }

   public static void applyDeadband(FrameVector2DBasics vectorToPack, double deadband)
   {
      double length = vectorToPack.length();
      if (length < deadband)
      {
         vectorToPack.setToZero();
      }
      else
      {
         double newLength = length - deadband;
         vectorToPack.scale(newLength / length);
      }
   }

   public static void applyDeadband(FrameVector3DBasics vectorToPack, double deadband)
   {
      double length = vectorToPack.length();
      if (length < deadband)
      {
         vectorToPack.setToZero();
      }
      else
      {
         double newLength = length - deadband;
         vectorToPack.scale(newLength / length);
      }
   }

   public static void applyDeadband(FramePoint2DBasics pointToPack, FramePoint2DReadOnly centerPoint, double deadband)
   {
      double distance = pointToPack.distance(centerPoint);
      if (distance < deadband)
      {
         pointToPack.set(centerPoint);
      }
      else
      {
         double newDistance = distance - deadband;
         pointToPack.interpolate(centerPoint, 1.0 - newDistance / distance);
      }
   }
}
