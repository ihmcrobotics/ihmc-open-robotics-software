package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

   private final PlanarRegionSnapTools snapTools;

   public SimplePlanarRegionFootstepNodeSnapper()
   {
      this(new DefaultFootstepPlannerParameters());
   }

   public SimplePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters)
   {
      super(parameters);

      snapTools = new PlanarRegionSnapTools(parameters);

      if(parameters.getProjectInsideDistance() > 0.001 + 0.5 * FootstepNode.gridSizeXY)
      {
         throw new RuntimeException("Projection distance is too big. Must be smaller than half of the grid size");
      }
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
         double y = yIndex * FootstepNode.gridSizeXY + projectionTranslation.getY();
         double z = highestRegion.getPlaneZGivenXY(x, y);

         RigidBodyTransform regionTransform = new RigidBodyTransform();
         highestRegion.getTransformToWorld(regionTransform);
         Quaternion regionOrientation = new Quaternion();
         regionTransform.getRotation(regionOrientation);

         RigidBodyTransform snapTransform = new RigidBodyTransform();
         snapTransform.setTranslation(projectionTranslation.getX(), projectionTranslation.getY(), z);
//         snapTransform.setRotation(regionOrientation);

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
}