package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class FootstepNodePlanarRegionSnapAndWiggler extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final DoubleProvider projectionInsideDelta;

   private final PlanarRegionSnapTools snapTools;

   public FootstepNodePlanarRegionSnapAndWiggler(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                                boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);
      this.projectionInsideDelta = projectionInsideDelta;

      snapTools = new PlanarRegionSnapTools(enforceTranslationLessThanGridCell);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      this.snapTools.setPlanarRegionsList(planarRegionsList, projectionInsideDelta.getValue(), parameters.getProjectInsideUsingConvexHull());
   }

   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      PlanarRegion highestPlanarRegion = snapTools.findHighestRegion(footPosition, new Vector2D());

      if (highestPlanarRegion == null)
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform snapTransform = PlanarRegionSnapTools.getSnapTransformToRegion(footPosition, highestPlanarRegion);

         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();

         return doSnapAndWiggle(footPosition, highestPlanarRegion, snapTransform);
      }
   }

   private FootstepNodeSnapData doSnapAndWiggle(Point2DReadOnly footPosition, PlanarRegion regionToWiggleIn, RigidBodyTransform snapTransform)
   {
      Point3D footPosition3D = new Point3D(footPosition);
      regionToWiggleIn.transformFromWorldToLocal(footPosition3D);
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footPosition3D);
      footPolygon.update();

      RigidBodyTransform wiggleTransformLocalToLocal = getWiggleTransformInPlanarRegionFrame(footPolygon, regionToWiggleIn);

      if (wiggleTransformLocalToLocal == null)
         return FootstepNodeSnapData.emptyData();

      RigidBodyTransform wiggleTransformWorldToWorld = getWiggleTransformInWorldFrame(wiggleTransformLocalToLocal, regionToWiggleIn);
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

      return new FootstepNodeSnapData(snapAndWiggleTransform);
   }


   private RigidBodyTransform getWiggleTransformInPlanarRegionFrame(ConvexPolygon2D footholdPolygon, PlanarRegion regionToWiggleInto)
   {
      updateWiggleParameters();

//      if (parameters.getProjectInsideUsingConvexHull())
         return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footholdPolygon, regionToWiggleInto, wiggleParameters);
//      else
//         return PolygonWiggler.wigglePolygonIntoRegion(footholdPolygon, regionToWiggleInto, wiggleParameters);
   }

   private static RigidBodyTransform getWiggleTransformInWorldFrame(RigidBodyTransform wiggleTransformLocalToLocal, PlanarRegion regionToWiggleInto)
   {
      RigidBodyTransform wiggleTransformWorldToWorld = new RigidBodyTransform();
      RigidBodyTransform localToWorld = new RigidBodyTransform();
      regionToWiggleInto.getTransformToWorld(localToWorld);
      RigidBodyTransform worldToLocal = new RigidBodyTransform(localToWorld);
      worldToLocal.invert();

      wiggleTransformWorldToWorld.set(localToWorld);
      wiggleTransformWorldToWorld.multiply(wiggleTransformLocalToLocal);
      wiggleTransformWorldToWorld.multiply(worldToLocal);
      return wiggleTransformWorldToWorld;
   }

   private void updateWiggleParameters()
   {
      wiggleParameters.deltaInside = projectionInsideDelta.getValue();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = 0.0;
      wiggleParameters.minYaw = -0.0;
   }
}
