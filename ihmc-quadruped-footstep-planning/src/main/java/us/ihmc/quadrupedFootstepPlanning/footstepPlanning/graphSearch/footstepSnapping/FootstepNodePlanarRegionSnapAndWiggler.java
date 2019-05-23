package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class FootstepNodePlanarRegionSnapAndWiggler extends FootstepNodeSnapper
{
   private final Point2D footPosition = new Point2D();

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final DoubleProvider projectionInsideDelta;
   private final BooleanProvider projectInsideUsingConvexHull;
   private final PlanarRegionConstraintDataHolder constraintDataHolder = new PlanarRegionConstraintDataHolder();
   private final PlanarRegionConstraintDataParameters constraintDataParameters = new PlanarRegionConstraintDataParameters();

   public FootstepNodePlanarRegionSnapAndWiggler(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                                 BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);
      this.projectionInsideDelta = projectionInsideDelta;
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;

      constraintDataParameters.enforceTranslationLessThanGridCell = enforceTranslationLessThanGridCell;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      constraintDataHolder.clear();
      constraintDataParameters.projectInsideUsingConvexHull = projectInsideUsingConvexHull.getValue();
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();
   }

   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      PlanarRegion highestPlanarRegion = PlanarRegionSnapTools.findHighestRegionWithProjection(footPosition, new Vector2D(), constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                                                                 constraintDataParameters);

      return snapFromPointInWorld(footPosition, highestPlanarRegion);


   }

   private FootstepNodeSnapData snapFromPointInWorld(Point2DReadOnly footPosition, PlanarRegion highestPlanarRegion)
   {
      if (highestPlanarRegion == null)
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform snapTransform = PlanarRegionSnapTools.getSnapTransformToRegion(footPosition, highestPlanarRegion);

         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();


         snapTransform = doSnapAndWiggle(footPosition, highestPlanarRegion, snapTransform);
         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();

         Point3D pointInWorld = new Point3D(footPosition);
         snapTransform.transform(pointInWorld);

         PlanarRegion newRegion = PlanarRegionSnapTools.findHighestRegionWithProjection(pointInWorld.getX(), pointInWorld.getY(),  new Vector2D(), constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                                                         constraintDataParameters);

         if (newRegion == null)
            return FootstepNodeSnapData.emptyData();

         if (!newRegion.equals(highestPlanarRegion))
         {
            return snapFromPointInWorld(new Point2D(pointInWorld), newRegion);
         }

         return new FootstepNodeSnapData(snapTransform);
      }
   }

   private RigidBodyTransform doSnapAndWiggle(Point2DReadOnly footPosition, PlanarRegion regionToWiggleIn, RigidBodyTransform snapTransform)
   {
      Point3D footPosition3D = new Point3D(footPosition);
      regionToWiggleIn.transformFromWorldToLocal(footPosition3D);
      Point2D footPositionInLocal = new Point2D(footPosition3D);

      RigidBodyTransform wiggleTransformLocalToLocal = getWiggleTransformInPlanarRegionFrame(footPositionInLocal, regionToWiggleIn);

      if (wiggleTransformLocalToLocal == null)
         return null;

      RigidBodyTransform wiggleTransformWorldToWorld = getWiggleTransformInWorldFrame(wiggleTransformLocalToLocal, regionToWiggleIn);
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform(wiggleTransformWorldToWorld);
      snapAndWiggleTransform.multiply(snapTransform);

      return snapAndWiggleTransform;
   }


   private RigidBodyTransform getWiggleTransformInPlanarRegionFrame(Point2DReadOnly footPositionInLocal, PlanarRegion regionToWiggleInto)
   {
      updateWiggleParameters();

      ConvexPolygon2D footholdPolygon = new ConvexPolygon2D();
      footholdPolygon.addVertex(footPositionInLocal);
      footholdPolygon.update();

      if (constraintDataParameters.projectInsideUsingConvexHull)
      {
         return PolygonWiggler.findWiggleTransform(footholdPolygon, regionToWiggleInto.getConvexHull(), wiggleParameters);
      }
      else
      {
         ConvexPolygon2DReadOnly containingRegion = PlanarRegionSnapTools.getContainingConvexRegion(footPositionInLocal, regionToWiggleInto.getConvexPolygons());
         if (containingRegion == null)
         {
            containingRegion = regionToWiggleInto.getConvexHull();
         }
         TIntArrayList indicesToExclude = constraintDataHolder.getIndicesToExclude(regionToWiggleInto, containingRegion, constraintDataParameters);
         return PolygonWiggler.findWiggleTransform(footholdPolygon, containingRegion, wiggleParameters, indicesToExclude.toArray());
      }
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
