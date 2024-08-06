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
}
