package us.ihmc.communication.packets;

import static java.lang.Double.isInfinite;
import static java.lang.Double.isNaN;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;

public abstract class ObjectValidityChecker
{

   public enum ObjectErrorType
   {
      NULL, CONTAINS_NaN, CONTAINS_INFINITY, INVALID_TIME_VALUE, WRONG_ARRAY_SIZE, NOT_NORMALIZED;

      private int expectedArraySize, actualArraySize;

      public void setExpectedArraySize(int expectedArraySize)
      {
         this.expectedArraySize = expectedArraySize;
      }

      public void setActualArraySize(int actualArraySize)
      {
         this.actualArraySize = actualArraySize;
      }

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
         case WRONG_ARRAY_SIZE:
            return "has a different size than expected. Expected size: " + expectedArraySize + ", actual size: " + actualArraySize;
         case NOT_NORMALIZED:
            return "Indicates that something isn't normalized, e.g. a non-unit Quaternion.";
         default:
            throw new RuntimeException("Enum value not handled: " + this);
         }
      }
   }

   /**
    * Checks the validity of a {@link Tuple2DBasics}
    * @param tuple2dToCheck
    * @return null if the tuple2d is valid, or the error message.
    */
   public static ObjectErrorType validateTuple2d(Tuple2DBasics tuple2dToCheck)
   {
      if (tuple2dToCheck == null)
         return ObjectErrorType.NULL;
      if (isNaN(tuple2dToCheck.getX()) || isNaN(tuple2dToCheck.getY()))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(tuple2dToCheck.getX()) || isInfinite(tuple2dToCheck.getY()))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   /**
    * Checks the validity of a {@link Tuple3DBasics}
    * @param tuple3dToCheck
    * @return null if the tuple3d is valid, or the error message.
    */
   public static ObjectErrorType validateTuple3d(Tuple3DBasics tuple3dToCheck)
   {
      if (tuple3dToCheck == null)
         return ObjectErrorType.NULL;
      if (isNaN(tuple3dToCheck.getX()) || isNaN(tuple3dToCheck.getY()) || isNaN(tuple3dToCheck.getZ()))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(tuple3dToCheck.getX()) || isInfinite(tuple3dToCheck.getY()) || isInfinite(tuple3dToCheck.getZ()))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   /**
    * Checks the validity of a {@link Tuple4DBasics}
    * @param tuple4dToCheck
    * @return null if the tuple4d is valid, or the error message.
    */
   public static ObjectErrorType validateTuple4d(Tuple4DBasics tuple4dToCheck)
   {
      if (tuple4dToCheck == null)
         return ObjectErrorType.NULL;
      if (isNaN(tuple4dToCheck.getX()) || isNaN(tuple4dToCheck.getY()) || isNaN(tuple4dToCheck.getZ()) || isNaN(tuple4dToCheck.getS()))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(tuple4dToCheck.getX()) || isInfinite(tuple4dToCheck.getY()) || isInfinite(tuple4dToCheck.getZ()) || isInfinite(tuple4dToCheck.getS()))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   public static ObjectErrorType validateQuat4d(Quaternion quat4dToCheck)
   {
      ObjectErrorType objectErrorType = validateTuple4d(quat4dToCheck);
      if(objectErrorType != null)
      {
         return objectErrorType;
      }
      else
      {
         boolean quaternionNormalized = quat4dToCheck.isUnitary(1.0e-5);
         if(!quaternionNormalized)
         {
            return ObjectErrorType.NOT_NORMALIZED;
         }
         else
         {
            return null;
         }
      }
   }

   /**
    * Checks the validity of a {@link AxisAngle}
    * @param axisAngleToCheck
    * @return null if the axisAngle4d is valid, or the error message.
    */
   public static ObjectErrorType validateAxisAngle4d(AxisAngle axisAngleToCheck)
   {
      if (axisAngleToCheck == null)
         return ObjectErrorType.NULL;
      if (isNaN(axisAngleToCheck.getX()) || isNaN(axisAngleToCheck.getY()) || isNaN(axisAngleToCheck.getZ()) || isNaN(axisAngleToCheck.getAngle()))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(axisAngleToCheck.getX()) || isInfinite(axisAngleToCheck.getY()) || isInfinite(axisAngleToCheck.getZ())
            || isInfinite(axisAngleToCheck.getAngle()))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   /**
    * Checks the validity of a {@link double}
    * @param doubleToCheck
    * @return null if the double is valid, or the error message.
    */
   public static ObjectErrorType validateDouble(double doubleToCheck)
   {
      if (isNaN(doubleToCheck))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(doubleToCheck))
         return ObjectErrorType.CONTAINS_INFINITY;

      return null;
   }

   /**
    * Checks the validity of a trajectoryTime
    * @param trajectoryTimeToCheck
    * @return null if the trajectoryTime is valid, or the error message.
    */
   public static ObjectErrorType validateTrajectoryTime(double trajectoryTimeToCheck)
   {
      if (isNaN(trajectoryTimeToCheck))
         return ObjectErrorType.CONTAINS_NaN;
      if (isInfinite(trajectoryTimeToCheck))
         return ObjectErrorType.CONTAINS_INFINITY;
      if (trajectoryTimeToCheck < 0.0)
         return ObjectErrorType.INVALID_TIME_VALUE;

      return null;
   }

   /**
    * Checks the validity of an array of double
    * @param doubleArrayToCheck
    * @param expectedSize
    * @return null if the array is of the expected size and if each element of the array is valid, or return the error message.
    */
   public static ObjectErrorType validateArrayOfDouble(double[] doubleArrayToCheck, int expectedSize)
   {
      if (doubleArrayToCheck.length != expectedSize)
      {
         ObjectErrorType wrongArraySize = ObjectErrorType.WRONG_ARRAY_SIZE;
         wrongArraySize.setActualArraySize(doubleArrayToCheck.length);
         wrongArraySize.setExpectedArraySize(expectedSize);
         return wrongArraySize;
      }
      for (int arrayIndex = 0; arrayIndex < doubleArrayToCheck.length; arrayIndex++)
      {
         ObjectErrorType validateDouble = validateDouble(doubleArrayToCheck[arrayIndex]);
         if (validateDouble != null)
            return validateDouble;
      }

      return null;
   }

   /**
    * Checks the validity of an enum
    * @param enumToCheck
    * @return null if the enum is valid, or the error message.
    */
   public static ObjectErrorType validateEnum(Enum<?> enumToCheck)
   {
      if (enumToCheck == null)
         return ObjectErrorType.NULL;

      return null;
   }
}
