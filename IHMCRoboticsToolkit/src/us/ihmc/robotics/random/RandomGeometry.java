package us.ihmc.robotics.random;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import Jama.Matrix;
import us.ihmc.commons.RandomNumbers;
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

/**
 * TODO: Rename to RandomGeometry or something.
 */
public class RandomGeometry
{
   public static Point3D nextPoint3D(Random random, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      double x = RandomNumbers.nextDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = RandomNumbers.nextDouble(random, -maxAbsoluteY, maxAbsoluteY);
      double z = RandomNumbers.nextDouble(random, -maxAbsoluteZ, maxAbsoluteZ);

      return new Point3D(x, y, z);
   }

   public static Point3D nextPoint3D(Random random, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      double x = RandomNumbers.nextDouble(random, minX, maxX);
      double y = RandomNumbers.nextDouble(random, minY, maxY);
      double z = RandomNumbers.nextDouble(random, minZ, maxZ);

      return new Point3D(x, y, z);
   }

   public static Point3D nextPoint3DWithEdgeCases(Random random, double probabilityForEdgeCase)
   {
      double x = RandomNumbers.nextDouble(random, probabilityForEdgeCase);
      double y = RandomNumbers.nextDouble(random, probabilityForEdgeCase);
      double z = RandomNumbers.nextDouble(random, probabilityForEdgeCase);
   
      return new Point3D(x, y, z);
   }

   public static Point3D nextPoint3D(Random random, double min, double max)
   {
      double x = RandomNumbers.nextDouble(random, min, max);
      double y = RandomNumbers.nextDouble(random, min, max);
      double z = RandomNumbers.nextDouble(random, min, max);

      return new Point3D(x, y, z);
   }

   public static Point3D nextPoint3D(Random random, double[] min, double[] max)
   {
      double x = RandomNumbers.nextDouble(random, min[0], max[0]);
      double y = RandomNumbers.nextDouble(random, min[1], max[1]);
      double z = RandomNumbers.nextDouble(random, min[2], max[2]);

      return new Point3D(x, y, z);
   }

   public static Point3D nextPoint3D(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      double x = RandomNumbers.nextDouble(random, min.getX(), max.getX());
      double y = RandomNumbers.nextDouble(random, min.getY(), max.getY());
      double z = RandomNumbers.nextDouble(random, min.getZ(), max.getZ());

      return new Point3D(x, y, z);
   }

   public static Point3D32 nextPoint3D32(Random random, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      float x = RandomNumbers.nextFloat(random, min.getX32(), max.getX32());
      float y = RandomNumbers.nextFloat(random, min.getY32(), max.getY32());
      float z = RandomNumbers.nextFloat(random, min.getZ32(), max.getZ32());

      return new Point3D32(x, y, z);
   }

   public static Point3D32[] nextPoint3D32Array(Random random, int size, Tuple3DReadOnly min, Tuple3DReadOnly max)
   {
      Point3D32[] randomPoint3fCloud = new Point3D32[size];

      for (int i = 0; i < randomPoint3fCloud.length; i++)
      {
         randomPoint3fCloud[i] = nextPoint3D32(random, min, max);
      }

      return randomPoint3fCloud;
   }

   public static Point2D nextPoint2D(Random random, double maxAbsoluteX, double maxAbsoluteY)
   {
      double x = RandomNumbers.nextDouble(random, -maxAbsoluteX, maxAbsoluteX);
      double y = RandomNumbers.nextDouble(random, -maxAbsoluteY, maxAbsoluteY);

      return new Point2D(x, y);
   }

   public static Point2D nextPoint2D(Random random, double minX, double minY, double maxX, double maxY)
   {
      double x = RandomNumbers.nextDouble(random, minX, maxX);
      double y = RandomNumbers.nextDouble(random, minY, maxY);

      return new Point2D(x, y);
   }

   public static Point2D32 nextPoint2D32(Random random, float maxAbsoluteX, float maxAblsoluteY)
   {
      float x = RandomNumbers.nextFloat(random, -maxAbsoluteX, maxAbsoluteX);
      float y = RandomNumbers.nextFloat(random, -maxAblsoluteY, maxAblsoluteY);

      return new Point2D32(x, y);
   }

   public static Point2D32 nextPoint2D32(Random random, float minX, float minY, float maxX, float maxY)
   {
      float x = RandomNumbers.nextFloat(random, minX, maxX);
      float y = RandomNumbers.nextFloat(random, minY, maxY);

      return new Point2D32(x, y);
   }

   public static Vector3D nextVector3D(Random random, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      double x = RandomNumbers.nextDouble(random, minX, maxX);
      double y = RandomNumbers.nextDouble(random, minY, maxY);
      double z = RandomNumbers.nextDouble(random, minZ, maxZ);

      return new Vector3D(x, y, z);
   }

   public static Vector3D nextVector3D(Random random, Tuple3DReadOnly boundary1, Tuple3DReadOnly boundary2)
   {
      Vector3D ret = new Vector3D();

      ret.setX(RandomNumbers.nextDouble(random, boundary1.getX(), boundary2.getX()));
      ret.setY(RandomNumbers.nextDouble(random, boundary1.getY(), boundary2.getY()));
      ret.setZ(RandomNumbers.nextDouble(random, boundary1.getZ(), boundary2.getZ()));

      return ret;
   }

   public static Vector3D nextVector3D(Random random)
   {
      return new Vector3D(random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
   }

   public static Vector3D nextOrthogonalVector3D(Random random, Vector3DReadOnly vectorToBeOrthogonalTo, boolean normalize)
   {
      Vector3D v1 = new Vector3D(vectorToBeOrthogonalTo.getY(), -vectorToBeOrthogonalTo.getX(), 0.0);
      Vector3D v2 = new Vector3D(-vectorToBeOrthogonalTo.getZ(), 0.0, vectorToBeOrthogonalTo.getX());

      Vector3D randomPerpendicular = new Vector3D();
      double a = RandomNumbers.nextDouble(random, 1.0);
      double b = RandomNumbers.nextDouble(random, 1.0);
      randomPerpendicular.scaleAdd(a, v1, randomPerpendicular);
      randomPerpendicular.scaleAdd(b, v2, randomPerpendicular);

      if (normalize)
         randomPerpendicular.normalize();

      return randomPerpendicular;
   }

   public static Vector3D32 nextVector3D32(Random random)
   {
      return new Vector3D32(nextVector3D(random));
   }

   public static Vector3D nextVector3D(Random random, double length)
   {
      Vector3D ret = nextVector3D(random);
      ret.normalize();
      ret.scale(length);

      return ret;
   }

   public static Vector3D[] nextVector3DArray(Random random, int numberOfVectors, double length)
   {
      Vector3D[] vectors = new Vector3D[numberOfVectors];
      for (int i = 0; i < numberOfVectors; i++)
         vectors[i] = nextVector3D(random, length);
      return vectors;
   }

   public static Vector4D nextVector4D(Random random, Tuple4DReadOnly lowerBound, Tuple4DReadOnly upperBound)
   {
      Vector4D ret = new Vector4D();

      ret.setX(RandomNumbers.nextDouble(random, lowerBound.getX(), upperBound.getX()));
      ret.setY(RandomNumbers.nextDouble(random, lowerBound.getY(), upperBound.getY()));
      ret.setZ(RandomNumbers.nextDouble(random, lowerBound.getZ(), upperBound.getZ()));
      ret.setS(RandomNumbers.nextDouble(random, lowerBound.getS(), upperBound.getS()));

      return ret;
   }

   public static Vector4D32 nextVector4D32(Random random, Tuple4DReadOnly lowerBound, Tuple4DReadOnly upperBound)
   {
      Vector4D32 ret = new Vector4D32();

      ret.setX(RandomNumbers.nextFloat(random, lowerBound.getX32(), upperBound.getX32()));
      ret.setY(RandomNumbers.nextFloat(random, lowerBound.getY32(), upperBound.getY32()));
      ret.setZ(RandomNumbers.nextFloat(random, lowerBound.getZ32(), upperBound.getZ32()));
      ret.setS(RandomNumbers.nextFloat(random, lowerBound.getS32(), upperBound.getS32()));

      return ret;
   }

   public static Vector2D nextVector2D(Random random)
   {
      return new Vector2D(random.nextDouble() - 0.5, random.nextDouble() - 0.5);
   }

   public static Vector2D nextVector2D(Random random, double length)
   {
      Vector2D ret = nextVector2D(random);
      ret.normalize();
      ret.scale(length);

      return ret;
   }

   public static Matrix3D nextDiagonalMatrix3D(Random random)
   {
      Matrix3D ret = new Matrix3D();
      ret.setM00(random.nextDouble());
      ret.setM11(random.nextDouble());
      ret.setM22(random.nextDouble());

      return ret;
   }

   public static AxisAngle nextAxisAngle(Random random)
   {
      return nextAxisAngle(random, Math.PI);
   }

   public static AxisAngle nextAxisAngle(Random random, double minMaxAngleRange)
   {
      // Generate uniformly random point on unit sphere (based on http://mathworld.wolfram.com/SpherePointPicking.html )
      double height = RandomNumbers.nextDouble(random, -1.0, 1.0);
      double angle = RandomNumbers.nextDouble(random, -minMaxAngleRange, minMaxAngleRange);
      double radius = Math.sqrt(1.0 - height * height);

      return new AxisAngle(radius * Math.cos(angle), radius * Math.sin(angle), height, angle);
   }

   public static Matrix nextJamaMatrix(Random random, int columns, int rows)
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

   public static Matrix3D nextMatrix3D(Random random, double maxAbsolute)
   {
      Matrix3D ret = new Matrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            ret.setElement(row, column, RandomNumbers.nextDouble(random, maxAbsolute));
         }
      }

      return ret;
   }

   public static DenseMatrix64F nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns)
   {
      return nextDenseMatrix64F(random, numberOfRows, numberOfColumns, 1.0);
   }

   public static DenseMatrix64F nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns, double maxAbsoluteValue)
   {
      DenseMatrix64F matrixToReturn = new DenseMatrix64F(numberOfRows, numberOfColumns);

      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            double value = RandomNumbers.nextDouble(random, maxAbsoluteValue);
            matrixToReturn.set(row, column, value);
         }
      }
      return matrixToReturn;
   }

   public static DenseMatrix64F nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns, double boundaryOne, double boundaryTwo)
   {
      DenseMatrix64F matrixToReturn = new DenseMatrix64F(numberOfRows, numberOfColumns);

      for (int row = 0; row < numberOfRows; row++)
      {
         for (int column = 0; column < numberOfColumns; column++)
         {
            double value = RandomNumbers.nextDouble(random, boundaryOne, boundaryTwo);
            matrixToReturn.set(row, column, value);
         }
      }
      return matrixToReturn;
   }

   public static Quaternion nextQuaternion(Random random)
   {
      return nextQuaternion(random, Math.PI);
   }

   public static Quaternion32 nextQuaternion32(Random random)
   {
      return new Quaternion32(nextQuaternion(random, Math.PI));
   }

   public static Quaternion nextQuaternion(Random random, double minMaxAngleRange)
   {
      AxisAngle orientation = nextAxisAngle(random, minMaxAngleRange);
      Quaternion quat = new Quaternion();
      quat.set(orientation);
      return quat;
   }

   public static RotationMatrix nextRotationMatrix(Random random)
   {
      Quaternion quaternion = nextQuaternion(random);
      RotationMatrix ret = new RotationMatrix();
      ret.set(quaternion);

      return ret;
   }
}
