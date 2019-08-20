package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.HashMap;

public class PawNodePlanarRegionSnapAndWiggler extends PawNodeSnapper
{
   private final Point2D pawPosition = new Point2D();

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final DoubleProvider projectionInsideDelta;
   private final BooleanProvider projectInsideUsingConvexHull;
   private final PlanarRegionPawConstraintDataHolder constraintDataHolder = new PlanarRegionPawConstraintDataHolder();
   private final PlanarRegionPawConstraintDataParameters constraintDataParameters = new PlanarRegionPawConstraintDataParameters();

   public PawNodePlanarRegionSnapAndWiggler(PawPlannerParametersReadOnly parameters, DoubleProvider projectionInsideDelta,
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
   public PawNodeSnapData snapInternal(int xIndex, int yIndex)
   {
      PawNodeTools.getPawPosition(xIndex, yIndex, pawPosition);
      constraintDataParameters.projectionInsideDelta = Math.min(projectionInsideDelta.getValue(), 0.025);
      PlanarRegion highestPlanarRegion = PlanarRegionPawSnapTools.findHighestRegionWithProjection(pawPosition, new Vector2D(), constraintDataHolder,
                                                                                                  planarRegionsList.getPlanarRegionsAsList(),
                                                                                                  constraintDataParameters);
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();

      originalRegion = highestPlanarRegion;
      regionsChecked.clear();


      return snapFromPointInWorld(pawPosition, highestPlanarRegion);


   }

   private PawNodeSnapData snapFromPointInWorld(Point2DReadOnly pawPositionInWorld, PlanarRegion highestPlanarRegion)
   {
      if (highestPlanarRegion == null)
      {
         return PawNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform snapTransform = PlanarRegionPawSnapTools.getSnapTransformToRegion(pawPositionInWorld, highestPlanarRegion);
         regionsChecked.put(highestPlanarRegion, snapTransform);

         if (snapTransform == null)
            return PawNodeSnapData.emptyData();


         snapTransform = doSnapAndWiggle(pawPositionInWorld, highestPlanarRegion);
         if (snapTransform == null)
            return PawNodeSnapData.emptyData();

         Point3D snappedPointInWorld = new Point3D(pawPositionInWorld);
         snapTransform.transform(snappedPointInWorld);

         constraintDataParameters.projectionInsideDelta = Math.min(projectionInsideDelta.getValue(), 0.025);
         PlanarRegion newRegion = PlanarRegionPawSnapTools.findHighestRegionWithProjection(snappedPointInWorld.getX(), snappedPointInWorld.getY(), new Vector2D(),
                                                                                           constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                                                                           constraintDataParameters);
         constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();


         if (newRegion == null)
            return PawNodeSnapData.emptyData();

         if (!newRegion.equals(highestPlanarRegion))
         {
            if (regionsChecked.containsKey(newRegion))
               return new PawNodeSnapData(regionsChecked.get(originalRegion));

            return snapFromPointInWorld(new Point2D(snappedPointInWorld), newRegion);
         }

         return new PawNodeSnapData(snapTransform);
      }
   }

   private RigidBodyTransform doSnapAndWiggle(Point2DReadOnly pawPositionInWorld, PlanarRegion regionToWiggleIn)
   {
      Point3D pawPositionInWorld3D = new Point3D(pawPositionInWorld);
      regionToWiggleIn.transformFromWorldToLocal(pawPositionInWorld3D);
      Point2D pawPositionInLocal = new Point2D(pawPositionInWorld3D);

      Point2DReadOnly wiggledPointInLocal = getWiggledPointInPlanarRegionFrame(pawPositionInLocal, regionToWiggleIn);

      if (wiggledPointInLocal == null)
         return null;

      Point3D wiggledPointInWorld = new Point3D(wiggledPointInLocal);
      regionToWiggleIn.transformFromLocalToWorld(wiggledPointInWorld);

      Quaternion regionOrientation = new Quaternion();
      regionToWiggleIn.transformFromLocalToWorld(regionOrientation);

      return PawNodeTools.computeSnapTransform(pawPositionInWorld, wiggledPointInWorld, regionOrientation);
   }

   private Point2DReadOnly getWiggledPointInPlanarRegionFrame(Point2DReadOnly pawPositionInLocal, PlanarRegion regionToWiggleInto)
   {
      updateWiggleParameters();

      ConvexPolygon2D pawPolygon = new ConvexPolygon2D();
      pawPolygon.addVertex(pawPositionInLocal);
      pawPolygon.update();

      if (constraintDataParameters.projectInsideUsingConvexHull)
      {
         return PolygonWiggler.wigglePolygon(pawPolygon, regionToWiggleInto.getConvexHull(), wiggleParameters).getCentroid();
      }
      else
      {
         ConvexPolygon2DReadOnly containingRegion = PlanarRegionPawSnapTools.getContainingConvexRegion(pawPositionInLocal, regionToWiggleInto.getConvexPolygons());
         if (containingRegion == null)
         {
            containingRegion = regionToWiggleInto.getConvexHull();
         }
         TIntArrayList indicesToExclude = constraintDataHolder.getIndicesToExclude(regionToWiggleInto, containingRegion, constraintDataParameters);
         return PolygonWiggler.wigglePolygon(pawPolygon, containingRegion, wiggleParameters, indicesToExclude.toArray()).getCentroid();
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
