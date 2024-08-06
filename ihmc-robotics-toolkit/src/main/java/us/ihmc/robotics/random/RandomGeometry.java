package us.ihmc.robotics.random;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;

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

   public static Vector3D nextVector3D(Random random, double length)
   {
      Vector3D ret = nextVector3D(random);
      ret.normalize();
      ret.scale(length);

      return ret;
   }

   public static DMatrixRMaj nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns)
   {
      return nextDenseMatrix64F(random, numberOfRows, numberOfColumns, 1.0);
   }

   public static DMatrixRMaj nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns, double maxAbsoluteValue)
   {
      DMatrixRMaj matrixToReturn = new DMatrixRMaj(numberOfRows, numberOfColumns);

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

   public static DMatrixRMaj nextDenseMatrix64F(Random random, int numberOfRows, int numberOfColumns, double boundaryOne, double boundaryTwo)
   {
      DMatrixRMaj matrixToReturn = new DMatrixRMaj(numberOfRows, numberOfColumns);

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
}
