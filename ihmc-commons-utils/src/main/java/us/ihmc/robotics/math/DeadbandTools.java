package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class DeadbandTools
{
   /**
    * Applies a deadband of size {@param deadbandSize} to value {@param value}, centered about zero.
    * <p>
    * For example, if a deadband of 1 is applied to a signal of 3, the return is 2. If a deadband of 3 is applied to a signal of 3, the return is 0.
    * </p>
    *
    * @param deadbandSize size of deadband to apply
    * @param value        value to apply the deadband to.
    * @return value after applying the deadband.
    */
   public static double applyDeadband(double deadbandSize, double value)
   {
      return applyDeadband(deadbandSize, 0.0, value);
   }

   /**
    * Applies a deadband of size {@param deadbandSize} to value {@param value}, centered about {@param deadbandCenter}.
    * <p>
    * For example, if a deadband of 1 is applied to a signal of 3 about 2, the return is 2. if a deadband of 3 is applied to a signal of 3 about 1, the return
    * is 1.
    * </p>
    *
    * @param deadbandSize   size of deadband to apply
    * @param deadbandCenter center about which the deadband is applied.
    * @param value          value to apply the deadband to.
    * @return value after applying the deadband.
    */
   public static double applyDeadband(double deadbandSize, double deadbandCenter, double value)
   {
      if (value > deadbandCenter)
         return Math.max(deadbandCenter, value - deadbandSize);
      else
         return Math.min(deadbandCenter, value + deadbandSize);
   }

   /**
    * Applies a deadband to the length of a vector.
    *
    * @param vectorToPack vector to apply the deadband to.
    * @param deadband     deadband magnitude to apply.
    */
   public static void applyDeadband(Vector2DBasics vectorToPack, double deadband)
   {
      double length = vectorToPack.norm();
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

   /**
    * Applies a deadband to the length of a vector.
    *
    * @param vectorToPack vector to apply the deadband to.
    * @param deadband     deadband magnitude to apply.
    */
   public static boolean applyDeadband(Vector3DBasics vectorToPack, double deadband)
   {
      double length = vectorToPack.norm();
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

   /**
    * Applies a deadband to the magnitude of a point from another point, where the total distance is the value that is deadbanded. This can be thought of as
    * applying a deadband of the size {@param deadband} to the vector from {@param centerPoint} to {@param pointToPack}, and then using this deadbanded vector
    * to recompute the value of {@param pointToPack}.
    *
    * @param pointToPack point to apply the deadband to.
    * @param centerPoint point about which to apply the deadband.
    * @param deadband    deadband magnitude to apply.
    */
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

   /**
    * Applies a deadband to the magnitude of a point from another point, where the total distance is the value that is deadbanded. This can be thought of as
    * applying a deadband of the size {@param deadband} to the vector from {@param centerPoint} to {@param pointToPack}, and then using this deadbanded vector
    * to recompute the value of {@param pointToPack}.
    *
    * @param pointToPack point to apply the deadband to.
    * @param centerPoint point about which to apply the deadband.
    * @param deadband    deadband magnitude to apply.
    */
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
