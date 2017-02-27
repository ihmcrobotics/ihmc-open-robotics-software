package us.ihmc.robotics.random;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import Jama.Matrix;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

public class RandomTools
{
   public static boolean generateRandomBoolean(Random random, double percentChanceTrue)
   {
      double percent = random.nextDouble();
      if (percent < percentChanceTrue)
         return true;
      return false;
   }

   public static int generateRandomInt(Random random, int boundaryOneInclusive, int boundaryTwoInclusive)
   {
      return boundaryOneInclusive + random.nextInt(boundaryTwoInclusive - boundaryOneInclusive + 1);
   }

   public static double generateRandomDouble(Random random, double maxAbsolute)
   {
      return generateRandomDouble(random, -maxAbsolute, maxAbsolute);
   }

   public static double generateRandomDouble(Random random, double boundaryOne, double boundaryTwo)
   {
      return boundaryOne + random.nextDouble() * (boundaryTwo - boundaryOne);
   }

   public static Point3D generateRandomPoint(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = generateRandomDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D(x, y, z);
   }

   public static Point3D generateRandomPoint(Random random, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      double x = generateRandomDouble(random, minX, maxX);
      double y = generateRandomDouble(random, minY, maxY);
      double z = generateRandomDouble(random, minZ, maxZ);

      return new Point3D(x, y, z);
   }

   public static Point3D generateRandomPoint3d(Random random, double min, double max)
   {
      double x = generateRandomDoubleInRange(random, min, max);
      double y = generateRandomDoubleInRange(random, min, max);
      double z = generateRandomDoubleInRange(random, min, max);

      return new Point3D(x, y, z);
   }

   public static Point3D generateRandomPoint3d(Random random, double[] min, double[] max)
   {
      double x = generateRandomDoubleInRange(random, min[0], max[0]);
      double y = generateRandomDoubleInRange(random, min[1], max[1]);
      double z = generateRandomDoubleInRange(random, min[2], max[2]);

      return new Point3D(x, y, z);
   }

   public static Point3D generateRandomPoint3d(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      double x = generateRandomDoubleInRange(random, min.getX(), max.getX());
      double y = generateRandomDoubleInRange(random, min.getY(), max.getY());
      double z = generateRandomDoubleInRange(random, min.getZ(), max.getZ());

      return new Point3D(x, y, z);
   }

   public static Point3D32 generateRandomPoint3f(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      float x = generateRandomFloatInRange(random, min.getX32(), max.getX32());
      float y = generateRandomFloatInRange(random, min.getY32(), max.getY32());
      float z = generateRandomFloatInRange(random, min.getZ32(), max.getZ32());

      return new Point3D32(x, y, z);
   }

   public static Point3D32[] generateRandomPoint3fCloud(Random random, int size, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      Point3D32[] randomPoint3fCloud = new Point3D32[size];

      for (int i = 0; i < randomPoint3fCloud.length; i++)
      {
         randomPoint3fCloud[i] = generateRandomPoint3f(random, min, max);
      }

      return randomPoint3fCloud;
   }

   public static Point2D generateRandomPoint2d(Random random, double maxAbsoluteX, double maxAbsoluteY)
   {
      double x = generateRandomDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = generateRandomDouble(random, -maxAbsoluteY, maxAbsoluteY);

      return new Point2D(x, y);
   }

   public static Point2D generateRandomPoint2d(Random random, double minX, double minY, double maxX, double maxY)
   {
      double x = generateRandomDouble(random, minX, maxX);
      double y = generateRandomDouble(random, minY, maxY);

      return new Point2D(x, y);
   }

   public static Point2D32 generateRandomPoint2f(Random random, float maxAbsoluteX, float maxAblsoluteY)
   {
      float x = generateRandomFloatInRange(random, -maxAbsoluteX, maxAbsoluteX);
      float y = generateRandomFloatInRange(random, -maxAblsoluteY, maxAblsoluteY);

      return new Point2D32(x, y);
   }

   public static Point2D32 generateRandomPoint2f(Random random, float minX, float minY, float maxX, float maxY)
   {
      float x = generateRandomFloatInRange(random, minX, maxX);
      float y = generateRandomFloatInRange(random, minY, maxY);

      return new Point2D32(x, y);
   }

   public static Vector3D generateRandomVector(Random random, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      double x = generateRandomDouble(random, minX, maxX);
      double y = generateRandomDouble(random, minY, maxY);
      double z = generateRandomDouble(random, minZ, maxZ);

      return new Vector3D(x, y, z);
   }

   public static Vector3D generateRandomVector(Random random, Tuple3DReadOnly boundary1, Tuple3DReadOnly boundary2)
   {
      Vector3D ret = new Vector3D();

      ret.setX(generateRandomDouble(random, boundary1.getX(), boundary2.getX()));
      ret.setY(generateRandomDouble(random, boundary1.getY(), boundary2.getY()));
      ret.setZ(generateRandomDouble(random, boundary1.getZ(), boundary2.getZ()));

      return ret;
   }

   public static Vector3D generateRandomVector(Random random)
   {
      return new Vector3D(random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
   }

   public static Vector3D generateRandomOrthogonalVector3d(Random random, Vector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector3D v1 = new Vector3D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX(), 0.0);
      Vector3D v2 = new Vector3D(-vectorToBeOrthogonalTo.getZ(), 0.0, vectorToBeOrthogonalTo.getX());

      Vector3D randomPerpendicular = new Vector3D();
      double a = generateRandomDouble(random, 1.0);
      double b = generateRandomDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);
      randomPerpendicular.scaleAdd(b, v2, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   public static Vector3D32 generateRandomVector3f(Random random)
   {
      return new Vector3D32(generateRandomVector(random));
   }

   public static Vector3D generateRandomVector(Random random, double length)
   {
      Vector3D ret = generateRandomVector(random);
      ret.normalize();
      ret.scale(length);

      return ret;
   }

   public static Vector3D[] generateRandomVectorArray(Random random, int numberOfVectors, double length)
   {
      Vector3D[] vectors = new Vector3D[numberOfVectors];
      for (int i = 0; i < numberOfVectors; i++)
         vectors[i] = generateRandomVector(random, length);
      return vectors;
   }

   public static Vector4D generateRandomVector4d(Random random, Tuple4DReadOnly lowerBound, Tuple4DReadOnly upperBound)
   {
      Vector4D ret = new Vector4D();

      ret.setX(generateRandomDouble(random, lowerBound.getX(), upperBound.getX()));
      ret.setY(generateRandomDouble(random, lowerBound.getY(), upperBound.getY()));
      ret.setZ(generateRandomDouble(random, lowerBound.getZ(), upperBound.getZ()));
      ret.setS(generateRandomDouble(random, lowerBound.getS(), upperBound.getS()));

      return ret;
   }

   public static Vector4D32 generateRandomVector4f(Random random, Tuple4DReadOnly lowerBound, Tuple4DReadOnly upperBound)
   {
      Vector4D32 ret = new Vector4D32();

      ret.setX(generateRandomFloatInRange(random, lowerBound.getX32(), upperBound.getX32()));
      ret.setY(generateRandomFloatInRange(random, lowerBound.getY32(), upperBound.getY32()));
      ret.setZ(generateRandomFloatInRange(random, lowerBound.getZ32(), upperBound.getZ32()));
      ret.setS(generateRandomFloatInRange(random, lowerBound.getS32(), upperBound.getS32()));

      return ret;
   }

   public static Vector2D generateRandomVector2d(Random random)
   {
      return new Vector2D(random.nextDouble() - 0.5, random.nextDouble() - 0.5);
   }

   public static Vector2D generateRandomVector2d(Random random, double length)
   {
      Vector2D ret = generateRandomVector2d(random);
      ret.normalize();
      ret.scale(length);

      return ret;
   }

   public static Matrix3D generateRandomDiagonalMatrix3d(Random random)
   {
      Matrix3D ret = new Matrix3D();
      ret.setM00(random.nextDouble());
      ret.setM11(random.nextDouble());
      ret.setM22(random.nextDouble());

      return ret;
   }

   public static AxisAngle generateRandomRotation(Random random)
   {
      return generateRandomRotation(random, Math.PI);
   }

   public static AxisAngle generateRandomRotation(Random random, double minMaxAngleRange)
   {
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html )
      double height = generateRandomDoubleInRange(random, -1.0, 1.0);
      double angle = generateRandomDoubleInRange(random, -minMaxAngleRange, minMaxAngleRange);
      double radius = Math.sqrt(1.0 - height * height);

      return new AxisAngle(radius * Math.cos(angle), radius * Math.sin(angle), height, angle);
   }

   public static double[] generateRandomDoubleArray(Random random, int length, double amplitude)
   {
      return generateRandomDoubleArray(random, length, -amplitude / 2.0, amplitude / 2.0);
   }

   public static double[] generateRandomDoubleArray(Random random, int length, double lowerBound, double upperBound)
   {
      double[] ret = new double[length];
      for (int i = 0; i < length; i++)
      {
         double parameter = random.nextDouble();
         ret[i] = parameter * lowerBound + (1.0 - parameter) * upperBound;
      }

      return ret;
   }

   public static float[] generateRandomFloatArray(Random random, int length, float amplitude)
   {
      return generateRandomFloatArray(random, length, -amplitude / 2.0f, amplitude / 2.0f);
   }

   public static float[] generateRandomFloatArray(Random random, int length, float lowerBound, float upperBound)
   {
      float[] ret = new float[length];
      for (int i = 0; i < length; i++)
      {
         float parameter = random.nextFloat();
         ret[i] = parameter * lowerBound + (1.0f - parameter) * upperBound;
      }

      return ret;
   }

   public static int[] generateRandomIntArray(Random random, int length, int amplitude)
   {
      int[] ret = new int[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = (int) Math.round((random.nextDouble() - 0.5) * 2.0 * amplitude);
      }

      return ret;
   }

   public static int[] generateRandomIntArray(Random random, int length, int lowerBound, int upperBound)
   {
      int[] ret = new int[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = (int) Math.round(random.nextDouble() * (upperBound - lowerBound) + lowerBound);
      }

      return ret;
   }

   public static Matrix generateRandomJamaMatrix(Random random, int columns, int rows)
   {
      Matrix ret = new Matrix(columns, rows);

      for (int i = 0; i < columns; i++)
      {
         for (int j = 0; j < rows; j++)
         {
            ret.set(i, j, random.nextDouble());
         }
      }

      return ret;
   }

   public static Matrix3D generateRandomMatrix3d(Random random, double maxAbsolute)
   {
      Matrix3D ret = new Matrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            ret.setElement(row, column, generateRandomDouble(random, maxAbsolute));
         }
      }

      return ret;
   }

   public static DenseMatrix64F generateRandomMatrix(Random random, int numberOfRows, int numberOfColumns)
   {
      return generateRandomMatrix(random, numberOfRows, numberOfColumns, 1.0);
   }

   public static DenseMatrix64F generateRandomMatrix(Random random, int numberOfRows, int numberOfColumns, double maxAbsoluteValue)
   {
      DenseMatrix64F matrixToReturn = new DenseMatrix64F(numberOfRows, numberOfColumns);

      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            double value = generateRandomDouble(random, maxAbsoluteValue);
            matrixToReturn.set(row, column, value);
         }
      }
      return matrixToReturn;
   }

   public static DenseMatrix64F generateRandomMatrix(Random random, int numberOfRows, int numberOfColumns, double boundaryOne, double boundaryTwo)
   {
      DenseMatrix64F matrixToReturn = new DenseMatrix64F(numberOfRows, numberOfColumns);

      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            double value = generateRandomDouble(random, boundaryOne, boundaryTwo);
            matrixToReturn.set(row, column, value);
         }
      }
      return matrixToReturn;
   }

   public static Quaternion generateRandomQuaternion(Random random)
   {
      return generateRandomQuaternion(random, Math.PI);
   }

   public static Quaternion32 generateRandomQuaternion4f(Random random)
   {
      return new Quaternion32(generateRandomQuaternion(random, Math.PI));
   }

   public static Quaternion generateRandomQuaternion(Random random, double minMaxAngleRange)
   {
      AxisAngle orientation = generateRandomRotation(random, minMaxAngleRange);
      Quaternion quat = new Quaternion();
      quat.set(orientation);
      return quat;
   }

   public static RotationMatrix generateRandomRotationMatrix(Random random)
   {
      Quaternion quaternion = generateRandomQuaternion(random);
      RotationMatrix ret = new RotationMatrix();
      ret.set(quaternion);

      return ret;
   }

   public static int generateRandomIntWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      int totalNumberOfInts = (int) (4 / probabilityForEdgeCase);

      int randInt = random.nextInt(totalNumberOfInts);
      switch (randInt)
      {
      case 0:
         return Integer.MIN_VALUE;
      case 1:
         return Integer.MAX_VALUE;
      case 2:
         return -Integer.MIN_VALUE;
      case 3:
         return -Integer.MAX_VALUE;
      default:
         return random.nextInt();
      }
   }

   public static double generateRandomDoubleWithEdgeCases(Random random, double probabilityForEdgeCase, double maxValue)
   {
      int totalNumberOfInts = (int) (7 / probabilityForEdgeCase);

      int randInt = random.nextInt(totalNumberOfInts);
      switch (randInt)
      {
      case 0:
         return Double.NaN;
      case 1:
         return Double.MIN_VALUE;
      case 2:
         return Double.MAX_VALUE;
      case 3:
         return -Double.MIN_VALUE;
      case 4:
         return -Double.MAX_VALUE;
      case 5:
         return Double.NEGATIVE_INFINITY;
      case 6:
         return Double.POSITIVE_INFINITY;
      default:
         return (random.nextDouble() * 2 - 1) * maxValue;
      }
   }

   public static double[] generateRandomDoubleArrayWithEdgeCases(Random random, int length, double probabilityForEdgeCase)
   {
      double[] ret = new double[length];
      for (int i = 0; i < length; i++)
      {
         ret[i] = generateRandomDoubleWithEdgeCases(random, probabilityForEdgeCase);
      }

      return ret;
   }

   public static Point3D generateRandomPointWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      double x = generateRandomDouble(random, probabilityForEdgeCase);
      double y = generateRandomDouble(random, probabilityForEdgeCase);
      double z = generateRandomDouble(random, probabilityForEdgeCase);

      return new Point3D(x, y, z);
   }

   public static double generateRandomDoubleWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      return generateRandomDoubleWithEdgeCases(random, probabilityForEdgeCase, Double.MAX_VALUE);
   }

   public static <T extends Enum<T>> T generateRandomEnum(Random random, Class<T> enumType)
   {
      int numberOfEnums = enumType.getEnumConstants().length;
      return enumType.getEnumConstants()[random.nextInt(numberOfEnums)];
   }

   /**
    * Returns an evenly distributed random number between two bounds. The bounds do not need to be
    * in min/max order.
    *
    * @param random Random number generator.
    * @param range1 One side of the bounds.
    * @param range2 Other side of the bounds.
    * @return Random number between bounds1 and bounds2.
    */
   public static double generateRandomDoubleInRange(Random random, double range1, double range2)
   {
      return range1 + (range2 - range1) * random.nextDouble();
   }

   public static float generateRandomFloatInRange(Random random, float range1, float range2)
   {
      return range1 + (range2 - range1) * random.nextFloat();
   }
}
