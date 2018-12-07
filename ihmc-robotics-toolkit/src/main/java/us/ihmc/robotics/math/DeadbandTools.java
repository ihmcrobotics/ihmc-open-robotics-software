package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

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

   public static void applyDeadband(Vector2DBasics vectorToPack, double deadband)
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

   public static boolean applyDeadband(Vector3DBasics vectorToPack, double deadband)
   {
      double length = vectorToPack.length();
      if (length < deadband)
      {
         vectorToPack.setToZero();
         return false;
      }
      else
      {
         double newLength = length - deadband;
         vectorToPack.scale(newLength / length);
         return true;
      }
   }

   public static boolean applyDeadband(Point2DBasics pointToPack, Point2DReadOnly centerPoint, double deadband)
   {
      double distance = pointToPack.distance(centerPoint);
      if (distance < deadband)
      {
         pointToPack.set(centerPoint);
         return false;
      }
      else
      {
         double newDistance = distance - deadband;
         pointToPack.interpolate(centerPoint, 1.0 - newDistance / distance);
         return true;
      }
   }

   public static boolean applyDeadband(Point3DBasics pointToPack, Point3DReadOnly centerPoint, double deadband)
   {
      double distance = pointToPack.distance(centerPoint);
      if (distance < deadband)
      {
         pointToPack.set(centerPoint);
         return false;
      }
      else
      {
         double newDistance = distance - deadband;
         pointToPack.interpolate(centerPoint, 1.0 - newDistance / distance);
         return true;
      }
   }
}
