package us.ihmc.commonWalkingControlModules.packetConsumers;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple4d;

public abstract class ObjectValidityChecker
{

   public enum ObjectErrorType
   {
      NULL, CONTAINS_NaN, CONTAINS_INFINITY, INVALID_TIME_VALUE;

      public String getMessage()
      {
         switch (this)
         {
            case CONTAINS_INFINITY:
               return "contains infinite value";
            case CONTAINS_NaN:
               return "contains NaN value";
            case INVALID_TIME_VALUE:
               return "is invalid time value";
            case NULL:
               return "is null";
            default:
               throw new RuntimeException("Enum value not handled: " + this );
         }
      }
   }

   public static ObjectErrorType validateTuple3d(Tuple3d tuple3dToCheck)
   {
      if (tuple3dToCheck == null)
         return ObjectErrorType.NULL;
      if (Double.isNaN(tuple3dToCheck.getX()) || Double.isNaN(tuple3dToCheck.getY()) || Double.isNaN(tuple3dToCheck.getZ()))
         return ObjectErrorType.CONTAINS_NaN;
      if (Double.isInfinite(tuple3dToCheck.getX()) || Double.isInfinite(tuple3dToCheck.getY()) || Double.isInfinite(tuple3dToCheck.getZ()))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   public static ObjectErrorType validateTuple4d(Tuple4d tuple4dToCheck)
   {
      if (tuple4dToCheck == null)
         return ObjectErrorType.NULL;
      if (Double.isNaN(tuple4dToCheck.getX()) || Double.isNaN(tuple4dToCheck.getY()) || Double.isNaN(tuple4dToCheck.getZ()) || Double.isNaN(tuple4dToCheck.w))
         return ObjectErrorType.CONTAINS_NaN;
      if (Double.isInfinite(tuple4dToCheck.getX()) || Double.isInfinite(tuple4dToCheck.getY()) || Double.isInfinite(tuple4dToCheck.getZ())
            || Double.isInfinite(tuple4dToCheck.w))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   public static ObjectErrorType validateAxisAngle4d(AxisAngle4d axisAngleToCheck)
   {
      if (axisAngleToCheck == null)
         return ObjectErrorType.NULL;
      if (Double.isNaN(axisAngleToCheck.getX()) || Double.isNaN(axisAngleToCheck.getY()) || Double.isNaN(axisAngleToCheck.getZ())
            || Double.isNaN(axisAngleToCheck.angle))
         return ObjectErrorType.CONTAINS_NaN;
      if (Double.isInfinite(axisAngleToCheck.getX()) || Double.isInfinite(axisAngleToCheck.getY()) || Double.isInfinite(axisAngleToCheck.getZ())
            || Double.isInfinite(axisAngleToCheck.angle))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   public static ObjectErrorType validateDouble(double doubleToCheck)
   {
      if (Double.isNaN(doubleToCheck))
         return ObjectErrorType.CONTAINS_NaN;
      if (Double.isInfinite(doubleToCheck))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   public static ObjectErrorType validateTrajectoryTime(double trajectoryTimeToCheck)
   {
      if (Double.isNaN(trajectoryTimeToCheck))
         return ObjectErrorType.CONTAINS_NaN;
      if (Double.isInfinite(trajectoryTimeToCheck))
         return ObjectErrorType.CONTAINS_INFINITY;
      if (trajectoryTimeToCheck < 0.0)
         return ObjectErrorType.INVALID_TIME_VALUE;

      return null;
   }

}
