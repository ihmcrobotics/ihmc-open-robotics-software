package us.ihmc.commons;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;

public class AngleTools
{

   public static final double PI = Math.PI;
   public static final double TwoPI = 2.0 * PI;
   public static final double EPSILON = 1e-10;

   private AngleTools()
   {

   }

   public static double trimAngleMinusPiToPi(double angle)
   {
      return AngleTools.shiftAngleToStartOfRange(angle, -PI);
   }

   public static float getAngle(Quaternion32 q)
   {
      return 2.0f * (float) Math.acos(q.getS());
   }

   public static double angleMinusPiToPi(Vector2DReadOnly startVector, Vector2DReadOnly endVector)
   {
      double absoluteAngle = Math.acos(startVector.dot(endVector) / startVector.norm() / endVector.norm());

      Vector3D start3d = new Vector3D(startVector.getX(), startVector.getY(), 0.0);
      Vector3D end3d = new Vector3D(endVector.getX(), endVector.getY(), 0.0);

      Vector3D crossProduct = new Vector3D();
      crossProduct.cross(start3d, end3d);

      if (crossProduct.getZ() >= 0.0)
      {
         return absoluteAngle;
      }
      else
      {
         return -absoluteAngle;
      }
   }

   public static double computeAngleAverage(double angleA, double angleB)
   {
      // average = A + (B-A)/2 = (A+B)/2
      return interpolateAngle(angleA, angleB, 0.5);
   }

   /**
    * Performs a linear interpolation from {@code angleA} to {@code angleB} given the percentage
    * {@code alpha} and trim the result to be in [-<i>pi</i>, <i>pi</i>].
    * <ul>
    * <li>If {@code alpha == 0}, the result is {@code angleA}.
    * <li>If {@code alpha == 1}, the result is {@code angleB}.
    * <li>If {@code alpha == 0.5}, the result is the average of the two angles.
    * <li>The percentage {@code alpha} is not clamped to be in [0, 1] such that this method can be used
    * for extrapolation.
    * </ul>
    * 
    * @param angleA the first angle in the interpolation.
    * @param angleB the second angle in the interpolation.
    * @param alpha  the percentage to use for the interpolation. A value of 0 will return
    *               {@code angleA}, while a value of 1 will return {@code angleB}.
    * @return the interpolated angle in [-<i>pi</i>, <i>pi</i>].
    */
   public static double interpolateAngle(double angleA, double angleB, double alpha)
   {
      // A + alpha * (B-A)/2
      double average = angleA + alpha * computeAngleDifferenceMinusPiToPi(angleB, angleA);
      return trimAngleMinusPiToPi(average);
   }

   /**
    * Formula found on <a href="https://en.wikipedia.org/wiki/Mean_of_circular_quantities"> Wikipedia</a>.
    */
   public static double computeAngleAverage(double[] angles)
   {
      double average = 0.0;

      double averageOfSin = 0.0;
      double averageOfCos = 0.0;
      double weight = 1.0 / angles.length;

      for (int i = 0; i < angles.length; i++)
      {
         averageOfSin += weight * Math.sin(angles[i]);
         averageOfCos += weight * Math.cos(angles[i]);
      }

      average = Math.atan2(averageOfSin, averageOfCos);

      return average;
   }

   /**
    * Finds the closest 90 degree yaw and returns number of 90 degrees (0 = 0; 1 = 90; 2 = 180; 3 = 270).
    *
    * @param yawInRadians double
    * @return int between 0 and 3 for the number of 90 degree yawed.
    */
   public static int findClosestNinetyDegreeYaw(double yawInRadians)
   {
      double minDifference = Double.POSITIVE_INFINITY;
      int ret = -1;

      double difference = AngleTools.computeAngleDifferenceMinusPiToPi(yawInRadians, 0.0);

      minDifference = Math.abs(difference);
      ret = 0;

      difference = AngleTools.computeAngleDifferenceMinusPiToPi(yawInRadians, Math.PI / 2.0);

      if (Math.abs(difference) < minDifference)
      {
         minDifference = Math.abs(difference);
         ret = 1;
      }

      difference = AngleTools.computeAngleDifferenceMinusPiToPi(yawInRadians, Math.PI);

      if (Math.abs(difference) < minDifference)
      {
         minDifference = Math.abs(difference);
         ret = 2;
      }

      difference = AngleTools.computeAngleDifferenceMinusPiToPi(yawInRadians, 3.0 * Math.PI / 2.0);

      if (Math.abs(difference) < minDifference)
      {
         minDifference = Math.abs(difference);
         ret = 3;
      }

      return ret;
   }

   /**
    * computeAngleDifferenceMinusPiToPi: returns (angleA - angleB), where the return value is [-pi, pi)
    *
    * @param angleA double
    * @param angleB double
    * @return double
    */
   public static double computeAngleDifferenceMinusPiToPi(double angleA, double angleB)
   {
      double difference = angleA - angleB;
      difference = difference % TwoPI;
      difference = AngleTools.shiftAngleToStartOfRange(difference, -PI);

      return difference;
   }

   /**
    * computeAngleDifferenceMinusPiToPi: returns (angleA - angleB), where the return value is [-2.0*pi, 0.0)
    *
    * @param angleA double
    * @param angleB double
    * @return double
    */
   public static double computeAngleDifferenceMinusTwoPiToZero(double angleA, double angleB)
   {
      double difference = angleA - angleB;
      difference = difference % TwoPI;
      difference = AngleTools.shiftAngleToStartOfRange(difference, -TwoPI);

      return difference;
   }

   /**
    * This will shift an angle to be in the range [<i>startOfAngleRange</i>,
    *  (<i>startOfAngleRange + 2*pi</i>)
    *
    * @param angleToShift      the angle to shift
    * @param startOfAngleRange start of the range.
    * @return the shifted angle
    */
   public static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange)
   {
      return shiftAngleToStartOfRange(angleToShift, startOfAngleRange, TwoPI);
   }

   /**
    * This will shift an angle to be in the range [<i>startOfAngleRange</i>,
    *  (<i>startOfAngleRange + endOfAngleRange</i>)
    *
    * @param angleToShift      the angle to shift
    * @param startOfAngleRange start of the range.
    * @param endOfAngleRange   end of the range.
    * @return the shifted angle
    */
   public static double shiftAngleToStartOfRange(double angleToShift, double startOfAngleRange, double endOfAngleRange)
   {
      double ret = angleToShift;
      startOfAngleRange = startOfAngleRange - EPSILON;

      if (angleToShift < startOfAngleRange)
      {
         ret = angleToShift + Math.ceil((startOfAngleRange - angleToShift) / endOfAngleRange) * endOfAngleRange;
      }

      if (angleToShift >= (startOfAngleRange + endOfAngleRange))
      {
         ret = angleToShift - Math.floor((angleToShift - startOfAngleRange) / endOfAngleRange) * endOfAngleRange;
      }

      return ret;
   }

   /**
    * Returns an evenly distributed random number between -2PI and 2PI
    *
    * @param random Random number generator
    * @return number between -2PI and 2PI
    */
   public static double generateRandomAngle(Random random)
   {
      return -2.0 * Math.PI + 4 * Math.PI * random.nextDouble();
   }

   /**
    * Returns array of angles increasing from -2PI to 2PI
    * 
    * @param numberOfAngles
    * @param stayThisFarAwayFromPlusMinus2PI
    * @param includeZero
    * @param includePlusMinusPI
    * @return
    */
   public static double[] generateArrayOfTestAngles(int numberOfAngles, double stayThisFarAwayFromPlusMinus2PI, boolean includeZero, boolean includePlusMinusPI)
   {
      int arraySize = numberOfAngles;
      double epsilon = stayThisFarAwayFromPlusMinus2PI;

      if (includeZero && !includePlusMinusPI)
      {
         arraySize += 1;
      }
      else if (includePlusMinusPI && !includeZero)
      {
         arraySize += 2;
      }
      else if (includeZero && includePlusMinusPI)
      {
         arraySize += 3;
      }

      double thetaMin = -2.0 * Math.PI + stayThisFarAwayFromPlusMinus2PI;
      double thetaMax = 2.0 * Math.PI - stayThisFarAwayFromPlusMinus2PI;
      double deltaTheta = Math.abs(thetaMax - thetaMin) / (numberOfAngles - 1);

      double[] ret = new double[arraySize];

      for (int i = 0; i < numberOfAngles; i++)
      {
         ret[i] = thetaMin + deltaTheta * i;

         boolean epsilonEqualToZero = MathTools.epsilonEquals(Math.abs(ret[i]), 0.0, epsilon);
         boolean epsilonEqualToPlusMinusPI = MathTools.epsilonEquals(Math.abs(ret[i]), Math.PI, epsilon);

         if (epsilonEqualToZero && !includeZero || epsilonEqualToPlusMinusPI && !includePlusMinusPI)
            ret[i] += Math.signum(ret[i]) * Math.max(1.0e-4, Math.abs(epsilon));
      }

      if (includeZero && !includePlusMinusPI)
      {
         ret[numberOfAngles] = 0.0;
      }
      else if (includePlusMinusPI && !includeZero)
      {
         ret[numberOfAngles] = -Math.PI;
         ret[numberOfAngles + 1] = Math.PI;
      }
      else if (includeZero && includePlusMinusPI)
      {
         ret[numberOfAngles] = 0.0;
         ret[numberOfAngles + 1] = -Math.PI;
         ret[numberOfAngles + 2] = Math.PI;
      }

      return ret;
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI. 
    * If the x or y components are both under the noTranslationTolerance, 
    * then the initial orientation as given in startPose will be returned. 
    *
    * @param startPose              initial position and orientation
    * @param endPoint               end position
    * @param headingOffset          offset from path angle
    * @param noTranslationTolerance tolerance for determining if path angle should be determined
    * @return number between -PI and PI
    */
   public static double calculateHeading(FramePose2DReadOnly startPose, FramePoint2DReadOnly endPoint, double headingOffset, double noTranslationTolerance)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();
      double heading;
      if (Math.abs(deltaX) < noTranslationTolerance && Math.abs(deltaY) < noTranslationTolerance)
      {
         heading = startPose.getYaw();
      }
      else
      {
         double pathHeading = Math.atan2(deltaY, deltaX);
         heading = AngleTools.trimAngleMinusPiToPi(pathHeading + headingOffset);
      }
      return heading;
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI.
    *
    * @param startPose              initial position and orientation
    * @param endPoint               end position
    * @param headingOffset          offset from path angle
    * @return number between -PI and PI
    */
   public static double calculateHeading(Tuple3DReadOnly startPose, Tuple3DReadOnly endPoint, double headingOffset)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();

      double pathHeading = Math.atan2(deltaY, deltaX);
      return AngleTools.trimAngleMinusPiToPi(pathHeading + headingOffset);
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI.
    *
    * @param startPose initial position
    * @param endPoint  end position
    * @return number between -PI and PI
    */
   public static double calculateHeading(Point2DReadOnly startPose, Point2DReadOnly endPoint)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();
      double heading;

      double pathHeading = Math.atan2(deltaY, deltaX);
      heading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      return heading;
   }

   /**
    * Pass in a vector. Get its angle in polar coordinates.
    * 
    * @param vx
    * @param vy
    * @return angle of vector from 0 to 2PI
    */
   public static double angleFromZeroToTwoPi(double vx, double vy)
   {
      double angleFromNegtivePiToPi = Math.atan2(vy, vx);

      if (angleFromNegtivePiToPi < 0.0)
      {
         return 2.0 * Math.PI + angleFromNegtivePiToPi;
      }
      else
      {
         return angleFromNegtivePiToPi;
      }
   }

   public static double roundToGivenPrecisionForAngle(double angleValue, double precisionFactor)
   {
      double centeredAngleValue = trimAngleMinusPiToPi(angleValue + 0.5 * precisionFactor);
      long longValue = (long) (centeredAngleValue / precisionFactor);
      double roundedValue = ((double) longValue) * precisionFactor;
      return trimAngleMinusPiToPi(roundedValue);
   }

   public static void roundToGivenPrecisionForAngles(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(roundToGivenPrecisionForAngle(tuple3d.getX(), precision));
      tuple3d.setY(roundToGivenPrecisionForAngle(tuple3d.getY(), precision));
      tuple3d.setZ(roundToGivenPrecisionForAngle(tuple3d.getZ(), precision));
   }
}
