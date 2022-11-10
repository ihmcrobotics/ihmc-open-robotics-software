package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConvexStepConstraintOptimizer;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.YoConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionFootstepSnapper implements FootstepAdjustment
{
   private final SteppableRegionsProvider steppableRegionsProvider;

   private final FramePose3D footstepAtSameHeightAsStanceFoot = new FramePose3D();
   private final FramePose3D adjustedFootstepPose = new FramePose3D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();

   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final YoConstraintOptimizerParameters wiggleParameters;
   private final YoBoolean useSimpleSnapping;

   private final ConvexStepConstraintOptimizer stepConstraintOptimizer;

   private PlanarRegionSnapVisualizer planarRegionSnapVisualizer;

   // temp variables used for calculation
   private final List<PlanarRegion> regionsIntersectingFoothold = new ArrayList<>();

   private final Point2D tempPoint = new Point2D();
   private final ConvexPolygon2D convexHullOfAllThePlanarRegions = new ConvexPolygon2D();

   private final PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame soleFrameAfterSnapAndBeforeWiggle = new PoseReferenceFrame("SoleFrameAfterSnapAndBeforeWiggle",
                                                                                               ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame soleFrameAfterWiggle = new PoseReferenceFrame("SoleFrameAfterWiggle", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame planarRegionFrame = new PoseReferenceFrame("PlanarRegionFrame", ReferenceFrame.getWorldFrame());

   // This is the transform that takes the foot pose from zero pitch, roll, and height, onto the planar region at that point.
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final PlanarRegion regionToSnapTo = new PlanarRegion();

   private final ConvexPolygon2D unsnappedFootstepPolygonInWorld = new ConvexPolygon2D();
   private final ConvexPolygon2D snappedAndWiggledFootstepPolygon = new ConvexPolygon2D();

   private final RigidBodyTransform transformFromSoleToRegion = new RigidBodyTransform();
   private final ConvexPolygon2D footPolygonInRegionFrame = new ConvexPolygon2D();

   public PlanarRegionFootstepSnapper(SideDependentList<ConvexPolygon2D> footPolygons,
                                      SteppableRegionsProvider steppableRegionsProvider,
                                      ConstraintOptimizerParametersReadOnly constraintOptimizerParameters,
                                      YoRegistry parentRegistry)
   {
      this.steppableRegionsProvider = steppableRegionsProvider;
      YoRegistry registry = new YoRegistry("PlanarRegionFootstepSnapper");
      this.wiggleParameters = new YoConstraintOptimizerParameters(constraintOptimizerParameters, registry);
      this.stepConstraintOptimizer = new ConvexStepConstraintOptimizer(registry);
      this.footPolygons = footPolygons;

      useSimpleSnapping = new YoBoolean("useSimpleSnapping", registry);
      useSimpleSnapping.set(true);

      wiggleParameters.setShouldPerformOptimization(false);
      wiggleParameters.setDesiredDistanceInside(-0.02);

      parentRegistry.addChild(registry);
   }

   public void attachPlanarRegionSnapVisualizer(PlanarRegionSnapVisualizer planarRegionSnapperCallback)
   {
      this.planarRegionSnapVisualizer = planarRegionSnapperCallback;
   }


   @Override
   public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose,
                                 FramePose2DReadOnly footstepPoseToAdjust,
                                 RobotSide footSide,
                                 FixedFramePose3DBasics adjustedPoseToPack)
   {
      footstepAtSameHeightAsStanceFoot.getPosition().set(footstepPoseToAdjust.getPosition());
      footstepAtSameHeightAsStanceFoot.setZ(stanceFootPose.getZ());
      footstepAtSameHeightAsStanceFoot.getOrientation().set(footstepPoseToAdjust.getOrientation());

      if (steppableRegionsProvider == null || steppableRegionsProvider.getSteppableRegions().isEmpty())
      {
         // we don't have any planar regions, so set the footstep pose height to match the stance foot height
         adjustedPoseToPack.set(footstepAtSameHeightAsStanceFoot);
         return false;
      }
      else
      {
         adjustedFootstepPose.set(footstepAtSameHeightAsStanceFoot);
         ConvexPolygon2DReadOnly footPolygonToWiggle = footPolygons.get(footSide);

         if (planarRegionSnapVisualizer != null)
         {
            planarRegionSnapVisualizer.recordUnadjustedFootstep(adjustedFootstepPose, footPolygonToWiggle);
         }

         boolean snapFailed;
         try
         {
            if (useSimpleSnapping.getValue())
            {
               snapTheFootStraightDown(adjustedFootstepPose, footPolygonToWiggle);
               // TODO we don't need to set the polygon yet.
//               wiggledPolygon.set(footPolygonToWiggle);
            }
            else
            {
               // TODO adding the wiggled polygon isn't necessary yet
               snapTheFootToRegionsAndWiggleInside(adjustedFootstepPose, footPolygonToWiggle);//, wiggledPolygon);
            }

            // check to make sure the snap didn't make the adjustment have NaN
            snapFailed = adjustedFootstepPose.containsNaN();
         }
         catch (RuntimeException e)
         {
            snapFailed = false;
         }

         // the adjustment results in a NaN or an exception, so use the footstep at the current height.
         if (snapFailed)
            adjustedPoseToPack.set(footstepAtSameHeightAsStanceFoot);
         else
            adjustedPoseToPack.set(adjustedFootstepPose);

         // return whether it's successful
         return !snapFailed;
      }
   }

   /**
    * Snaps the foot straight down onto the planar regions
    *
    * @param unsnappedSolePose
    * @param footStepPolygonInSoleFrame
    */
   private void snapTheFootStraightDown(FramePose3DBasics solePoseToSnap, ConvexPolygon2DReadOnly footStepPolygonInSoleFrame)
   {
      // If we project the foot vertically down, find all the planar regions that the foothold would intersect
      // FIXME this doesn't require any frames, you can just use the pose directly as a transform.
      soleFrameBeforeSnapping.setPoseAndUpdate(solePoseToSnap);
      unsnappedFootstepPolygonInWorld.set(footStepPolygonInSoleFrame);
      unsnappedFootstepPolygonInWorld.applyTransform(solePoseToSnap, false);

      PlanarRegionTools.findPlanarRegionsIntersectingPolygon(unsnappedFootstepPolygonInWorld,
                                                             steppableRegionsProvider.getSteppableRegions(),
                                                             regionsIntersectingFoothold);

      // If the foot isn't above any planar region, set the height to the closest region. If it is above some regions, set the height to the highest point if
      // vertically projecting
      FixedFramePoint3DBasics footPosition = solePoseToSnap.getPosition();
      PlanarRegion closestRegion;
      if (regionsIntersectingFoothold.size() == 0)
      {
         closestRegion = null;
         solePoseToSnap.set(footstepAtSameHeightAsStanceFoot);
      }
      else
      {
         // FIXME this very well may be bad, if the planar regions are at a ridiculous angle.
         closestRegion = findHighestPlanarRegionAtPoint(footPosition.getX(), footPosition.getY(), regionsIntersectingFoothold);
         // set the height of the footstep as the height on the closest region
         footPosition.setZ(closestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
      }

      // Update the visualizer
      if (planarRegionSnapVisualizer != null)
      {
         snapTransform.setToZero();
         snapTransform.getTranslation().setZ(footPosition.getZ());
         planarRegionSnapVisualizer.recordSnapTransform(regionsIntersectingFoothold.size(), snapTransform, closestRegion);
      }
   }

   private void snapTheFootToRegionsAndWiggleInside(FramePose3DBasics solePose,
                                                    ConvexPolygon2DReadOnly footStepPolygonInSoleFrame)
//                                                    ConvexPolygon2DBasics snappedFootstepPolygonToPack)
   {
      // If we project the foot vertically down, find all the planar regions that the foothold would intersect
      // FIXME this doesn't require any frames, you can just use the pose directly as a transform.
      soleFrameBeforeSnapping.setPoseAndUpdate(solePose);
      unsnappedFootstepPolygonInWorld.set(footStepPolygonInSoleFrame);
      unsnappedFootstepPolygonInWorld.applyTransform(solePose, false);

      if (isFootPolygonOnBoundaryOfPlanarRegions(steppableRegionsProvider.getSteppableRegions(), unsnappedFootstepPolygonInWorld))
      {
         PlanarRegionTools.findPlanarRegionsIntersectingPolygon(unsnappedFootstepPolygonInWorld,
                                                                steppableRegionsProvider.getSteppableRegions(),
                                                                regionsIntersectingFoothold);

         // If the foot isn't above any planar region, set the height to the closest region. If it is above some regions, set the height to the highest point if
         // vertically projecting
         FixedFramePoint3DBasics footPosition = solePose.getPosition();
         PlanarRegion closestRegion = null;
         if (regionsIntersectingFoothold.size() == 0)
         {
//            closestRegion = findClosestPlanarRegionToPointByProjectionOntoXYPlane(footPosition.getX(),
//                                                                                  footPosition.getY(),
//                                                                                  steppableRegionsProvider.getSteppableRegions());
            solePose.set(footstepAtSameHeightAsStanceFoot);
         }
         else
         {
            closestRegion = findHighestPlanarRegionAtPoint(footPosition.getX(), footPosition.getY(), regionsIntersectingFoothold);

            footPosition.setZ(closestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
         }
         // TODO we don't yet need to set the snapped polygon
//         snappedFootstepPolygonToPack.set(footStepPolygonInSoleFrame);

         if (planarRegionSnapVisualizer != null)
         {
            planarRegionSnapVisualizer.recordFootPoseIsOnBoundary();
            snapTransform.setToZero();
            snapTransform.getTranslation().setZ(footPosition.getZ());
            planarRegionSnapVisualizer.recordSnapTransform(regionsIntersectingFoothold.size(), snapTransform, closestRegion);
         }


         return;
      }


      // Snap the sole pose from where it was spawned to the planar region environment
      snapFootstepToPlanarRegionEnvironment(regionsIntersectingFoothold, solePose, regionToSnapTo, unsnappedFootstepPolygonInWorld);
      // Wiggle the snapped pose into the planar region environments
      wiggleFootstepIntoPlanarRegion(solePose, footStepPolygonInSoleFrame, regionToSnapTo);
      // Crop the foothold to match the region
      // TODO this isn't necessary yet, since we can't pack the polygons
//      cropFootholdToMatchRegion(solePose, footStepPolygonInSoleFrame, regionToSnapTo, snappedFootstepPolygonToPack);
   }



   private void snapFootstepToPlanarRegionEnvironment(List<PlanarRegion> regionsIntersectingFoothold,
                                                      FramePose3DBasics solePoseToSnap,
                                                      PlanarRegion regionToPack,
                                                      ConvexPolygon2DReadOnly footPolygonInWorld)
   {
      if (!snapper.snapPolygonToPlanarRegionsList(footPolygonInWorld,
                                                  regionsIntersectingFoothold,
                                                  Double.POSITIVE_INFINITY,
                                                  regionToPack,
                                                  snapTransform))
      {
         throw new RuntimeException("Snapping failed");
      }

      solePoseToSnap.setZ(0.0);
      solePoseToSnap.applyTransform(snapTransform);

      if (planarRegionSnapVisualizer != null)
         planarRegionSnapVisualizer.recordSnapTransform(regionsIntersectingFoothold.size(), snapTransform, regionToPack);
   }

   private void wiggleFootstepIntoPlanarRegion(FramePose3DBasics solePoseToWiggle,
                                               ConvexPolygon2DReadOnly footStepPolygonInSoleFrame,
                                               PlanarRegion planarRegionToWiggleInside)
   {
      if (!wiggleParameters.shouldPerformOptimization())
         return;

      planarRegionFrame.setPoseAndUpdate(planarRegionToWiggleInside.getTransformToWorld());
      soleFrameAfterSnapAndBeforeWiggle.setPoseAndUpdate(solePoseToWiggle);

      soleFrameAfterSnapAndBeforeWiggle.getTransformToDesiredFrame(transformFromSoleToRegion, planarRegionFrame);
      footPolygonInRegionFrame.set(footStepPolygonInSoleFrame);
      footPolygonInRegionFrame.applyTransform(transformFromSoleToRegion, false);

      stepConstraintOptimizer.reset();
      RigidBodyTransformReadOnly wiggleTransform = stepConstraintOptimizer.findConstraintTransform(footPolygonInRegionFrame,
                                                                                                   planarRegionToWiggleInside.getConvexHull(),
                                                                                                   wiggleParameters);

      if (wiggleTransform == null)
      { // Wiggle failed, set the pose to NaN
         solePoseToWiggle.setToNaN();
      }
      else
      {
         // change the footstep pose into the planar region, and then compute where it as after the transform
         solePoseToWiggle.changeFrame(planarRegionFrame);
         solePoseToWiggle.applyTransform(wiggleTransform);

         // get the pose of the footstep in the world
         solePoseToWiggle.changeFrame(ReferenceFrame.getWorldFrame());

         if (planarRegionSnapVisualizer != null)
            planarRegionSnapVisualizer.recordWiggleTransform(wiggleTransform);
      }
   }

   private void cropFootholdToMatchRegion(FramePose3DReadOnly solePose,
                                          ConvexPolygon2DReadOnly footStepPolygonInSoleFrame,
                                          PlanarRegion planarRegionSnappedTo,
                                          ConvexPolygon2DBasics snappedFootholdInSoleToPack)
   {
      // check for partial foothold
      if (wiggleParameters.shouldPerformOptimization() && wiggleParameters.getDesiredDistanceInside() < 0.0)
      {
         soleFrameAfterWiggle.setPoseAndUpdate(solePose);
         soleFrameAfterWiggle.getTransformToDesiredFrame(transformFromSoleToRegion, planarRegionFrame);
         footPolygonInRegionFrame.set(footStepPolygonInSoleFrame);
         footPolygonInRegionFrame.applyTransform(transformFromSoleToRegion, false);

         convexPolygonTools.computeIntersectionOfPolygons(planarRegionSnappedTo.getConvexHull(), footPolygonInRegionFrame, snappedFootholdInSoleToPack);

         // transform from the region to the world
         snappedFootholdInSoleToPack.applyInverseTransform(transformFromSoleToRegion, false);
      }
      else
      { // If we're not supposed to crop the foothold, set it to the value in the world
         snappedFootholdInSoleToPack.set(footStepPolygonInSoleFrame);
      }
   }

   /**
    * Checks if the foot polygon extends past the modeled environment.
    *
    * @return whether the footstep gets out of bounds of the modeled environment.
    */
   private boolean isFootPolygonOnBoundaryOfPlanarRegions(List<PlanarRegion> planarRegionsList, ConvexPolygon2DReadOnly footPolygonInWorld)
   {
      // get the convex hull of all the planar regions in the environment by adding all the convex hulls
      convexHullOfAllThePlanarRegions.clear();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion region = planarRegionsList.get(i);

         for (int j = 0; j < region.getConcaveHullSize(); j++)
         {
            region.getTransformToWorld().transform(region.getConcaveHullVertex(j), tempPoint, false);
            convexHullOfAllThePlanarRegions.addVertex(tempPoint);
         }
      }
      convexHullOfAllThePlanarRegions.update();

      // Check if the foot polygon is outside this convex hull. That hull represents the entire modeled world.
      for (int i = 0; i < footPolygonInWorld.getNumberOfVertices(); i++)
      {
         if (!convexHullOfAllThePlanarRegions.isPointInside(footPolygonInWorld.getVertex(i)))
            return true;
      }
      return false;
   }

   /**
    * Finds the planar region that is the closest distance in x-y to the point passed in.
    *
    * @param x              x position of the point in question in the world
    * @param y              y position of the point in question in the world
    * @param regionsToCheck regions to check for finding the height
    * @return closest region to the point
    */
   private static PlanarRegion findClosestPlanarRegionToPointByProjectionOntoXYPlane(double x, double y, List<PlanarRegion> regionsToCheck)
   {
      double shortestDistanceToPoint = Double.POSITIVE_INFINITY;
      PlanarRegion closestRegion = null;

      for (int i = 0; i < regionsToCheck.size(); i++)
      {
         PlanarRegion candidateRegion = regionsToCheck.get(i);
         double distanceToRegion = candidateRegion.distanceToPointByProjectionOntoXYPlane(x, y);
         if (distanceToRegion < shortestDistanceToPoint)
         {
            shortestDistanceToPoint = distanceToRegion;
            closestRegion = candidateRegion;
         }
      }

      return closestRegion;
   }

   /**
    * Finds the highest planar region at the point
    *
    * @param x                 x position of the point in question in the world
    * @param y                 y position of the point in question in the world
    * @param regionsBelowPoint regions below the x-y position passed in
    * @return highest region at the point
    */
   private static PlanarRegion findHighestPlanarRegionAtPoint(double x, double y, List<PlanarRegion> regionsBelowPoint)
   {
      double highestPoint = Double.NEGATIVE_INFINITY;
      PlanarRegion highestRegion = null;

      for (int i = 0; i < regionsBelowPoint.size(); i++)
      {
         PlanarRegion candidateRegion = regionsBelowPoint.get(i);
         double heightAtPoint = candidateRegion.getPlaneZGivenXY(x, y);
         if (heightAtPoint > highestPoint)
         {
            highestPoint = heightAtPoint;
            highestRegion = candidateRegion;
         }
      }

      return highestRegion;
   }
}
