package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.function.ToDoubleFunction;

public class FootstepSnapAndWiggler implements FootstepSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private double flatGroundHeight = 0.0;

   private final HashMap<DiscreteFootstep, FootstepSnapData> snapDataHolder = new HashMap<>();

   private final FootstepPlannerEnvironmentHandler environmentHandler;

   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private final HeightMapSnapWiggler heightMapSnapWiggler;

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters, FootstepPlannerEnvironmentHandler environmentHandler)
   {
      this(footPolygonsInSoleFrame, parameters, null, environmentHandler,null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 FootstepPlannerParametersReadOnly parameters,
                                 TickAndUpdatable tickAndUpdatable,
                                 FootstepPlannerEnvironmentHandler environmentHandler,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
      this.heightMapSnapWiggler = new HeightMapSnapWiggler(footPolygonsInSoleFrame, wiggleParameters);
      this.environmentHandler = environmentHandler;
   }

   public void setFlatGroundHeight(double flatGroundHeight)
   {
      this.flatGroundHeight = flatGroundHeight;
   }

   public void initialize()
   {
      updateWiggleParameters(wiggleParameters, parameters);
   }

   public void clearSnapData()
   {
      snapDataHolder.clear();
   }

   public FootstepSnapData snapFootstep(DiscreteFootstep footstep)
   {
      return snapFootstep(footstep, null, false);
   }

   @Override
   public FootstepSnapData snapFootstep(DiscreteFootstep footstep, DiscreteFootstep stanceStep, boolean computeWiggleTransform)
   {
      if (snapDataHolder.containsKey(footstep))
      {
         FootstepSnapData snapData = snapDataHolder.get(footstep);
         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (snapData.getWiggleTransformInWorld().containsNaN() && computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
         }

         return snapData;
      }
      else if (environmentHandler.flatGroundMode())
      {
         return FootstepSnapData.identityData(flatGroundHeight);
      }
      else
      {
         FootstepSnapData snapData = computeSnapTransform(footstep, stanceStep);
         snapDataHolder.put(footstep, snapData);

         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
         }

         return snapData;
      }
   }

   /**
    * Can manually add snap data for a footstep to bypass the snapper.
    */
   public void addSnapData(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      snapDataHolder.put(footstep, snapData);
   }

   protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
   {
      double maximumRegionHeightToConsider = getMaximumRegionHeightToConsider(stanceStep);
      DiscreteFootstepTools.getFootPolygon(footstepToSnap, footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()), footPolygon);
      boolean snappedToPlanarReigons = false;
      PlanarRegionsList planarRegionsListToUse = null;
      boolean snappedToHeightMap = true;
      RigidBodyTransform snapTransform = heightMapSnapper.snapPolygonToHeightMap(footPolygon, environmentHandler.getFallbackHeightMap(), parameters.getHeightMapSnapThreshold());

      if (snapTransform == null)
      {
         return FootstepSnapData.emptyData();
      }
      else
      {
         FootstepSnapData snapData = new FootstepSnapData(snapTransform);

         if (planarRegionsListToUse != null)
         {
            snapData.setRegionIndex(getIndex(planarRegionToPack, planarRegionsListToUse));
            computeCroppedFoothold(footstepToSnap, snapData);
         }
         if (snappedToHeightMap)
         {
            snapData.setRMSErrorHeightMap(heightMapSnapper.getNormalizedRMSError());
            snapData.setHeightMapArea(heightMapSnapper.getArea());
         }
         snapData.setSnappedToHeightMap(snappedToHeightMap);
         snapData.setSnappedToPlanarRegions(snappedToPlanarReigons);

         return snapData;
      }
   }

   private static int getIndex(PlanarRegion planarRegion, PlanarRegionsList planarRegionsList)
   {
      double epsilon = 1e-7;
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         if (planarRegionsList.getPlanarRegion(i).epsilonEquals(planarRegion, epsilon))
         {
            return i;
         }
      }

      return -1;
   }

   private double getMaximumRegionHeightToConsider(DiscreteFootstep stanceStep)
   {
      if (stanceStep == null)
      {
         return Double.POSITIVE_INFINITY;
      }

      FootstepSnapData snapData = snapDataHolder.get(stanceStep);
      if (snapData == null || snapData.getSnapTransform().containsNaN())
      {
         return Double.POSITIVE_INFINITY;
      }
      else
      {
         return parameters.getMaximumSnapHeight() + DiscreteFootstepTools.getSnappedStepHeight(stanceStep, snapData.getSnapTransform());
      }
   }

   protected void computeWiggleTransform(DiscreteFootstep footstepToWiggle, DiscreteFootstep stanceStep, FootstepSnapData snapData)
   {
      heightMapSnapWiggler.computeWiggleTransform(footstepToWiggle, environmentHandler.getFallbackHeightMap(), snapData, parameters.getHeightMapSnapThreshold());

      if (stanceStep != null && snapDataHolder.containsKey(stanceStep))
      {
         FootstepSnapData stanceStepSnapData = snapDataHolder.get(stanceStep);

         // check for overlap
         boolean overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
         }

         // check for overlap after this steps wiggle is removed. if still overlapping, remove wiggle on stance step
         overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            stanceStepSnapData.getWiggleTransformInWorld().setIdentity();
         }
      }

      computeCroppedFoothold(footstepToWiggle, snapData);
   }

   /**
    * Extracted to method for testing purposes
    */
   protected RigidBodyTransform wiggleIntoConvexHull(ConvexPolygon2D footPolygonInRegionFrame)
   {
      return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegionFrame, planarRegionToPack, wiggleParameters);
   }

   private final RigidBodyTransform transform1 = new RigidBodyTransform();
   private final RigidBodyTransform transform2 = new RigidBodyTransform();
   private final ConvexPolygon2D polygon1 = new ConvexPolygon2D();
   private final ConvexPolygon2D polygon2 = new ConvexPolygon2D();

   /**
    * Extracted to method for testing purposes
    */
   protected boolean stepsAreTooClose(DiscreteFootstep step1, FootstepSnapData snapData1, DiscreteFootstep step2, FootstepSnapData snapData2)
   {
      DiscreteFootstepTools.getFootPolygon(step1, footPolygonsInSoleFrame.get(step1.getRobotSide()), polygon1);
      DiscreteFootstepTools.getFootPolygon(step2, footPolygonsInSoleFrame.get(step2.getRobotSide()), polygon2);

      snapData1.packSnapAndWiggleTransform(transform1);
      snapData2.packSnapAndWiggleTransform(transform2);

      polygon1.applyTransform(transform1, false);
      polygon2.applyTransform(transform2, false);

      boolean intersection = StepConstraintPolygonTools.arePolygonsIntersecting(polygon1, polygon2);
      if (intersection)
      {
         return true;
      }

      double distance = StepConstraintPolygonTools.distanceBetweenPolygons(polygon1, polygon2);
      return distance < parameters.getMinClearanceFromStance();
   }

   protected void computeCroppedFoothold(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      if (environmentHandler.flatGroundMode())
      {
         snapData.getCroppedFoothold().clearAndUpdate();
         return;
      }

      DiscreteFootstepTools.getFootPolygon(footstep, footPolygonsInSoleFrame.get(footstep.getRobotSide()), footPolygon);

      snapData.packSnapAndWiggleTransform(tempTransform);
      ConvexPolygon2D snappedPolygonInWorld = FootstepSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);
      ConvexPolygon2D croppedFootPolygon = FootstepSnappingTools.computeRegionIntersection(planarRegionToPack, snappedPolygonInWorld);

      if (!croppedFootPolygon.isEmpty())
      {
         FootstepSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstep, tempTransform, croppedFootPolygon);
         snapData.getCroppedFoothold().set(croppedFootPolygon);
      }
   }

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, FootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
   }

   /**
    * package-private for testing
    */
   static double computeAchievedDeltaInside(ConvexPolygon2DReadOnly footPolygon, PlanarRegion planarRegion, boolean useConcaveHull)
   {
      double achievedDeltaInside = Double.POSITIVE_INFINITY;
      ToDoubleFunction<Point2DReadOnly> deltaInsideCalculator;

      if (useConcaveHull)
      {
         Vertex2DSupplier concaveHullVertices = Vertex2DSupplier.asVertex2DSupplier(planarRegion.getConcaveHull());
         deltaInsideCalculator = vertex ->
         {
            boolean pointIsInside = StepConstraintPolygonTools.isPointInsidePolygon(concaveHullVertices, vertex);
            double distanceSquaredFromPerimeter = FootPlacementConstraintCalculator.distanceSquaredFromPerimeter(concaveHullVertices, vertex, null);
            return (pointIsInside ? 1 : -1) * Math.sqrt(distanceSquaredFromPerimeter);
         };
      }
      else
      {
         deltaInsideCalculator = vertex -> -planarRegion.getConvexHull().signedDistance(vertex);
      }

      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         double insideDelta = deltaInsideCalculator.applyAsDouble(footPolygon.getVertex(i));
         if (insideDelta < achievedDeltaInside)
         {
            achievedDeltaInside = insideDelta;
         }
      }

      return achievedDeltaInside;
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      snapDataHolder.clear();
   }
}
