package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PolygonSnapperTools
{
   public static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3DReadOnly surfaceNormal)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      constructRotationToMatchSurfaceNormal(surfaceNormal, transformToReturn.getRotation());

      return transformToReturn;
   }

   /**
    * This computes the equivalent rotation matrix to match the provided surface normal. That is, a plane with the orientation of the rotation
    * {@param rotationTransformToPack} will have the same normal as {@param surfaceNormal}.
    * <p>
    * The provided surface normal does not necessarily have to be a unit vector.
    *
    * @param surfaceNormal           desired surface normal to achieve. Not modified.
    * @param rotationTransformToPack Rotation matrix to store the result. Modified.
    */
   public static void constructRotationToMatchSurfaceNormal(Vector3DReadOnly surfaceNormal, RotationMatrixBasics rotationTransformToPack)
   {
      double normalLengthSquared = surfaceNormal.normSquared();
      double scale = 1.0;
      if (normalLengthSquared > 1.0)
         scale = 1.0 / EuclidCoreTools.fastSquareRoot(normalLengthSquared);

      // Use these values to make the surface normal a unit vector
      double surfaceNormalX = surfaceNormal.getX() * scale;
      double surfaceNormalY = surfaceNormal.getY() * scale;
      double surfaceNormalZ = surfaceNormal.getZ() * scale;

      // xAxis = yAxisInWorld cross SurfaceNormal. yAxisInWorld = (0, 1, 0)
      double xAxisX = surfaceNormalZ;
      double xAxisY = 0.0;
      double xAxisZ = -surfaceNormalX;

      double xNormScale = 1.0 / EuclidCoreTools.norm(xAxisX, xAxisZ);

      xAxisX *= xNormScale;
      xAxisZ *= xNormScale;

      // yAxis = surfaceNormal cross xAxis
      double yAxisX = surfaceNormalY * xAxisZ;
      double yAxisY = surfaceNormalZ * xAxisX - surfaceNormalX * xAxisZ;
      double yAxisZ = -surfaceNormalY * xAxisX;

      rotationTransformToPack.set(xAxisX, yAxisX, surfaceNormalX, xAxisY, yAxisY, surfaceNormalY, xAxisZ, yAxisZ, surfaceNormalZ);
   }

   /**
    * Computes the transform's translation that will transform a point from (X, Y, 0) to the desired point at {@param highestVertex}, where (X, Y) are the same,
    * given that the transform already contains a rotation transformation. This rotation transformation is stored in the rotation component of
    * {@param transformToReturn}.
    * Note that this overrides the translation component of {@param transformToReturn}.
    */
   public static void setTranslationSettingZAndPreservingXAndY(Point3DReadOnly highestVertex, RigidBodyTransformBasics transformToReturn)
   {
      setTranslationSettingZAndPreservingXAndY(highestVertex, transformToReturn.getRotation(), transformToReturn.getTranslation());
   }

   /**
    * Computes the translation transform that will transform a point from (X, Y, 0) to the desired point at {@param highestVertex}, where (X, Y) are the same,
    * after also applying the provided rotation transform {@param rotationTransform}.
    */
   private static void setTranslationSettingZAndPreservingXAndY(Point3DReadOnly highestVertex,
                                                                Orientation3DReadOnly rotationTransform,
                                                                Tuple3DBasics translationTransformToPack)
   {
      setTranslationToAchieveZAndPreserveXAndY(highestVertex.getX(), highestVertex.getY(), 0.0, highestVertex.getZ(), rotationTransform, translationTransformToPack);
   }

   public static void setTranslationToAchieveZAndPreserveXAndY(Point2DReadOnly vertex,
                                                               double unsnappedHeight,
                                                               double desiredSnappedHeight,
                                                               Orientation3DReadOnly rotationTransform,
                                                               Tuple3DBasics translationTransformToPack)
   {
      setTranslationToAchieveZAndPreserveXAndY(vertex.getX(),
                                               vertex.getY(),
                                               unsnappedHeight,
                                               desiredSnappedHeight,
                                               rotationTransform,
                                               translationTransformToPack);
   }

   public static void setTranslationToAchieveZAndPreserveXAndY(double vertexX,
                                                               double vertexY,
                                                               double unsnappedHeight,
                                                               double desiredSnappedHeight,
                                                               Orientation3DReadOnly rotationTransform,
                                                               Tuple3DBasics translationTransformToPack)
   {
      // The operation of a transform is first to perform a rotation and then a translation. So we first must compute where we want the initial point to be
      // before the rotation is applied to achieve the desired height. Since we have the location of the point that we want to achieve at the end,

      // The initial height of the point being snapped will always start at zero height. So first, apply the rotation to that initial translation such
      translationTransformToPack.set(-vertexX, -vertexY, -unsnappedHeight);
      rotationTransform.transform(translationTransformToPack);

      // Now add the desired vertex, such that the translation is achieved. This is the same as appending
      translationTransformToPack.add(vertexX, vertexY, desiredSnappedHeight);
   }
}
