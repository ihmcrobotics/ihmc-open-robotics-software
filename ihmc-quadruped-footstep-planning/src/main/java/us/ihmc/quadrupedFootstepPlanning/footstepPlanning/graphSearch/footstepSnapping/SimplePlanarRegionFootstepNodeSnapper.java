package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

   private final PlanarRegionSnapTools snapTools;

   public SimplePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta)
   {
      super(parameters);

      snapTools = new PlanarRegionSnapTools(projectionInsideDelta);
   }

   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      Vector2D projectionTranslation = new Vector2D();
      PlanarRegion highestRegion = findHighestRegion(footPosition.getX(), footPosition.getY(), projectionTranslation);

      if(highestRegion == null || projectionTranslation.containsNaN() || isTranslationBiggerThanGridCell(projectionTranslation))
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         double x = xIndex * FootstepNode.gridSizeXY + projectionTranslation.getX();
         double xTranslated = x + projectionTranslation.getX();
         double y = yIndex * FootstepNode.gridSizeXY + projectionTranslation.getY();
         double yTranslated = y + projectionTranslation.getY();
         double z = highestRegion.getPlaneZGivenXY(xTranslated, yTranslated);

         Vector3D surfaceNormal = new Vector3D();
         highestRegion.getNormal(surfaceNormal);

         RigidBodyTransform snapTransform = createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
         setTranslationSettingZAndPreservingXAndY(x, y, xTranslated, yTranslated, z, snapTransform);

         return new FootstepNodeSnapData(snapTransform);
      }
   }

   private PlanarRegion findHighestRegion(double x, double y, Vector2D projectionTranslationToPack)
   {
      return snapTools.findHighestRegion(x, y, projectionTranslationToPack, planarRegionsList.getPlanarRegionsAsList());
   }

   private boolean isTranslationBiggerThanGridCell(Vector2D translation)
   {
      double maximumTranslationPerAxis = 0.5 * FootstepNode.gridSizeXY;
      return Math.abs(translation.getX()) > maximumTranslationPerAxis || Math.abs(translation.getY()) > maximumTranslationPerAxis;
   }


   private static RigidBodyTransform createTransformToMatchSurfaceNormalPreserveX(Vector3D surfaceNormal)
   {
      Vector3D xAxis = new Vector3D();
      Vector3D yAxis = new Vector3D(0.0, 1.0, 0.0);

      xAxis.cross(yAxis, surfaceNormal);
      xAxis.normalize();
      yAxis.cross(surfaceNormal, xAxis);

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setColumns(xAxis, yAxis, surfaceNormal);
      RigidBodyTransform transformToReturn = new RigidBodyTransform();
      transformToReturn.setRotation(rotationMatrix);
      return transformToReturn;
   }

   private static void setTranslationSettingZAndPreservingXAndY(double x, double y, double xTranslated, double yTranslated, double z, RigidBodyTransform transformToReturn)
   {
      Vector3D newTranslation = new Vector3D(x, y, 0.0);
      transformToReturn.transform(newTranslation);
      newTranslation.scale(-1.0);
      newTranslation.add(xTranslated, yTranslated, z);

      transformToReturn.setTranslation(newTranslation);
   }
}