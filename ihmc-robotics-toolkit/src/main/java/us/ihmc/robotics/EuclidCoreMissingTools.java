package us.ihmc.robotics;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class EuclidCoreMissingTools
{

   public static void floorToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.floorToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.floorToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.floorToPrecision(tuple3d.getZ(), precision));
      
   }

   public static void roundToGivenPrecision(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(MathTools.roundToPrecision(tuple3d.getX(), precision));
      tuple3d.setY(MathTools.roundToPrecision(tuple3d.getY(), precision));
      tuple3d.setZ(MathTools.roundToPrecision(tuple3d.getZ(), precision));
   }

   public static boolean isFinite(Tuple3DBasics tuple)
   {
      return Double.isFinite(tuple.getX()) && Double.isFinite(tuple.getY()) && Double.isFinite(tuple.getZ());
   }

   /**
    * Projects the provided {@code rotation} onto {@code axis} such that the original rotation can be decomposed
    * into a rotation around {@code axis} and one around an orthogonal axis.
    * <p>
    * rotation = orthogonalRotation * result
    * </p>
    * @param rotation is the original rotation to be projected onto {@code axis}
    * @param axis is the desired rotation axis of the result.
    * @param result will be modified to contain the component of {@code rotation} that is around {@code axis}
    */
   public static void projectRotationOnAxis(QuaternionReadOnly rotation, Vector3DReadOnly axis, QuaternionBasics result)
   {
      double dotProduct = rotation.getX() * axis.getX() + rotation.getY() * axis.getY() + rotation.getZ() * axis.getZ();

      double scale = dotProduct / axis.lengthSquared();
      double projectedX = scale * axis.getX();
      double projectedY = scale * axis.getY();
      double projectedZ = scale * axis.getZ();

      result.set(projectedX, projectedY, projectedZ, rotation.getS());
      result.normalize();
   }

}
