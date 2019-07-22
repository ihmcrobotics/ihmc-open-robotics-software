package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.HashMap;
import java.util.HashSet;

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

   private PlanarRegion originalRegion;
   private final HashMap<PlanarRegion, RigidBodyTransform> regionsChecked = new HashMap<>();
   @Override
   public FootstepNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      constraintDataParameters.projectionInsideDelta = Math.min(projectionInsideDelta.getValue(), 0.025);
      PlanarRegion highestPlanarRegion = PlanarRegionSnapTools.findHighestRegionWithProjection(footPosition, new Vector2D(), constraintDataHolder,
                                                                                               planarRegionsList.getPlanarRegionsAsList(),
                                                                                               constraintDataParameters);
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();

      originalRegion = highestPlanarRegion;
      regionsChecked.clear();


      return snapFromPointInWorld(footPosition, highestPlanarRegion);


   }

   private FootstepNodeSnapData snapFromPointInWorld(Point2DReadOnly footPositionInWorld, PlanarRegion highestPlanarRegion)
   {
      if (highestPlanarRegion == null)
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform snapTransform = PlanarRegionSnapTools.getSnapTransformToRegion(footPositionInWorld, highestPlanarRegion);
         regionsChecked.put(highestPlanarRegion, snapTransform);

         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();


         snapTransform = doSnapAndWiggle(footPositionInWorld, highestPlanarRegion);
         if (snapTransform == null)
            return FootstepNodeSnapData.emptyData();

         Point3D snappedPointInWorld = new Point3D(footPositionInWorld);
         snapTransform.transform(snappedPointInWorld);

         constraintDataParameters.projectionInsideDelta = Math.min(projectionInsideDelta.getValue(), 0.025);
         PlanarRegion newRegion = PlanarRegionSnapTools.findHighestRegionWithProjection(snappedPointInWorld.getX(), snappedPointInWorld.getY(),  new Vector2D(),
                                                                                        constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                                                                        constraintDataParameters);
         constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();


         if (newRegion == null)
            return FootstepNodeSnapData.emptyData();

         if (!newRegion.equals(highestPlanarRegion))
         {
            if (regionsChecked.containsKey(newRegion))
               return new FootstepNodeSnapData(regionsChecked.get(originalRegion));

            return snapFromPointInWorld(new Point2D(snappedPointInWorld), newRegion);
         }

         return new FootstepNodeSnapData(snapTransform);
      }
   }

   private RigidBodyTransform doSnapAndWiggle(Point2DReadOnly footPositionInWorld, PlanarRegion regionToWiggleIn)
   {
      Point3D footPositionInWorld3D = new Point3D(footPositionInWorld);
      regionToWiggleIn.transformFromWorldToLocal(footPositionInWorld3D);
      Point2D footPositionInLocal = new Point2D(footPositionInWorld3D);

      Point2DReadOnly wiggledPointInLocal = getWiggledPointInPlanarRegionFrame(footPositionInLocal, regionToWiggleIn);

      if (wiggledPointInLocal == null)
         return null;

      Point3D wiggledPointInWorld = new Point3D(wiggledPointInLocal);
      regionToWiggleIn.transformFromLocalToWorld(wiggledPointInWorld);

      Quaternion regionOrientation = new Quaternion();
      regionToWiggleIn.transformFromLocalToWorld(regionOrientation);

      return FootstepNodeTools.computeSnapTransform(footPositionInWorld, wiggledPointInWorld, regionOrientation);
   }

   private Point2DReadOnly getWiggledPointInPlanarRegionFrame(Point2DReadOnly footPositionInLocal, PlanarRegion regionToWiggleInto)
   {
      updateWiggleParameters();

      ConvexPolygon2D footholdPolygon = new ConvexPolygon2D();
      footholdPolygon.addVertex(footPositionInLocal);
      footholdPolygon.update();

      if (constraintDataParameters.projectInsideUsingConvexHull)
      {
         return PolygonWiggler.wigglePolygon(footholdPolygon, regionToWiggleInto.getConvexHull(), wiggleParameters).getCentroid();
      }
      else
      {
         ConvexPolygon2DReadOnly containingRegion = PlanarRegionSnapTools.getContainingConvexRegion(footPositionInLocal, regionToWiggleInto.getConvexPolygons());
         if (containingRegion == null)
         {
            containingRegion = regionToWiggleInto.getConvexHull();
         }
         TIntArrayList indicesToExclude = constraintDataHolder.getIndicesToExclude(regionToWiggleInto, containingRegion, constraintDataParameters);
         return PolygonWiggler.wigglePolygon(footholdPolygon, containingRegion, wiggleParameters, indicesToExclude.toArray()).getCentroid();
      }
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
