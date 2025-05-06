package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PolygonSnapperTools
{
   public static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3DReadOnly surfaceNormal)
   {
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      constructTransformToMatchSurfaceNormalPreserveX(surfaceNormal, transformToReturn);

      return transformToReturn;
   }

   public static void constructTransformToMatchSurfaceNormalPreserveX(Vector3DReadOnly surfaceNormal, RigidBodyTransform transformToPack)
   {
      // xAxis = yAxis cross SurfaceNormal
      double xAxisX = surfaceNormal.getZ();
      double xAxisY = 0.0;
      double xAxisZ = -surfaceNormal.getX();

      double xNorm = EuclidCoreTools.norm(xAxisX, xAxisZ);

      xAxisX /= xNorm;
      xAxisZ /= xNorm;

      // yAxis = surfaceNormal cross xAxis
      double yAxisX = surfaceNormal.getY() * xAxisZ;
      double yAxisY = surfaceNormal.getZ() * xAxisX - surfaceNormal.getX() * xAxisZ;
      double yAxisZ = -surfaceNormal.getY() * xAxisX;

      transformToPack.getRotation().set(xAxisX, yAxisX, surfaceNormal.getX(), xAxisY, yAxisY, surfaceNormal.getY(), xAxisZ, yAxisZ, surfaceNormal.getZ());
   }
}
