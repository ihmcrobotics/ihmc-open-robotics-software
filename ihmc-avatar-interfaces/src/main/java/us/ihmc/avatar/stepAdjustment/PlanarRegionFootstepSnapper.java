package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConvexStepConstraintOptimizer;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.YoConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.SteppableRegionsProvider;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class PlanarRegionFootstepSnapper implements FootstepAdjustment
{
   private final YoRegistry registry;

   private final SteppableRegionsProvider steppableRegionsProvider;

   private final FramePose3D footstepAtSameHeightAsStanceFoot = new FramePose3D();
   private final FramePose3D adjustedFootstepPose = new FramePose3D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();

   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;

   private final YoConstraintOptimizerParameters wiggleParameters;

   private final ConvexStepConstraintOptimizer stepConstraintOptimizer;
   protected final PlanarRegion regionToSnapTo = new PlanarRegion();

   private PlanarRegionSnapperCallback planarRegionSnapperCallback;

   public PlanarRegionFootstepSnapper(SideDependentList<ConvexPolygon2D> footPolygons, SteppableRegionsProvider steppableRegionsProvider)
   {
      this.steppableRegionsProvider = steppableRegionsProvider;
      registry = new YoRegistry("PlanarRegionFootstepSnapper");
      this.wiggleParameters = new YoConstraintOptimizerParameters(registry);
      this.stepConstraintOptimizer = new ConvexStepConstraintOptimizer(registry);
      this.footPolygons = footPolygons;

      wiggleParameters.setDesiredDistanceInside(0.02);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public GarbageFreePlanarRegionListPolygonSnapper getSnapper()
   {
      return snapper;
   }

   public void attachPlanarRegionSnapperCallback(PlanarRegionSnapperCallback planarRegionSnapperCallback)
   {
      this.planarRegionSnapperCallback = planarRegionSnapperCallback;
   }

   public ConvexPolygon2DReadOnly getFootPolygon(RobotSide robotSide)
   {
      return footPolygons.get(robotSide);
   }

   private final ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();
   private final ConvexPolygon2D wiggledPolygon = new ConvexPolygon2D();

   @Override
   public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FixedFramePose3DBasics adjustedPoseToPack)
   {
      footstepAtSameHeightAsStanceFoot.getPosition().set(footstepPose.getPosition());
      footstepAtSameHeightAsStanceFoot.setZ(stanceFootPose.getZ());
      footstepAtSameHeightAsStanceFoot.getOrientation().set(footstepPose.getOrientation());

      if (steppableRegionsProvider != null && !steppableRegionsProvider.getSteppableRegions().isEmpty())
      {
         adjustedFootstepPose.set(footstepAtSameHeightAsStanceFoot);
         footPolygonToWiggle.set(footPolygons.get(footSide));

         if (planarRegionSnapperCallback != null)
         {
            planarRegionSnapperCallback.advanceFootIndex();
            planarRegionSnapperCallback.recordUnadjustedFootstep(adjustedFootstepPose, footPolygonToWiggle);
         }

         try
         {
            snapAndWiggle(adjustedFootstepPose, footPolygonToWiggle, wiggledPolygon);
            if (adjustedFootstepPose.containsNaN())
            {
               adjustedPoseToPack.set(footstepAtSameHeightAsStanceFoot);
               return false;
            }
         }
         catch (RuntimeException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there are no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
            adjustedPoseToPack.set(footstepAtSameHeightAsStanceFoot);
         }
         adjustedPoseToPack.set(adjustedFootstepPose);
         return true;
      }
      else
      {
         adjustedPoseToPack.set(footstepAtSameHeightAsStanceFoot);
         return false;
      }
   }

   private final PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", ReferenceFrame.getWorldFrame());

   private final FrameConvexPolygon2D footstepPolygonInWorld = new FrameConvexPolygon2D();
   private final RigidBodyTransform transformToSole = new RigidBodyTransform();

   public boolean snapAndWiggle(FramePose3D solePose, ConvexPolygon2DReadOnly footStepPolygonInSoleFrame, ConvexPolygon2DBasics snappedFootstepPolygonToPack)
   {


      if (steppableRegionsProvider == null || steppableRegionsProvider.getSteppableRegions().isEmpty())
      {
         snappedFootstepPolygonToPack.clear();
         return false;
      }

      soleFrameBeforeSnapping.setPoseAndUpdate(solePose);
      footstepPolygonInWorld.setIncludingFrame(soleFrameBeforeSnapping, footStepPolygonInSoleFrame);
      footstepPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame()); // this works if the soleFrames are z up.

      if (isFootPolygonOnBoundaryOfPlanarRegions(steppableRegionsProvider.getSteppableRegions(), footstepPolygonInWorld))
      {
         /*
          * If foot is on the boundary of planar regions, don't snap/wiggle but
          * set it to the nearest plane's height
          */
         FixedFramePoint3DBasics footPosition = solePose.getPosition();
         PlanarRegion closestRegion = findClosestPlanarRegionToPointByProjectionOntoXYPlane(footPosition.getX(), footPosition.getY());
         footPosition.setZ(closestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
         snappedFootstepPolygonToPack.set(footStepPolygonInSoleFrame);

         if (planarRegionSnapperCallback != null)
            planarRegionSnapperCallback.recordFootPoseIsOnBoundary();

         return true;
      }

      doSnapAndWiggle(solePose, footStepPolygonInSoleFrame, footstepPolygonInWorld, snappedFootstepPolygonToPack);

      solePose.get(transformToSole);
      snappedFootstepPolygonToPack.applyInverseTransform(transformToSole, false);

      return true;
   }

   private final RigidBodyTransform snapTransform = new RigidBodyTransform();

   private final PoseReferenceFrame soleFrameAfterSnapAndBeforeWiggle = new PoseReferenceFrame("SoleFrameAfterSnapAndBeforeWiggle", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame soleFrameAfterWiggle = new PoseReferenceFrame("SoleFrameAfterWiggle", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame planarRegionFrame = new PoseReferenceFrame("PlanarRegionFrame", ReferenceFrame.getWorldFrame());

   private final RigidBodyTransform transformFromSoleToRegion = new RigidBodyTransform();
   private final ConvexPolygon2D footPolygonInRegionFrame = new ConvexPolygon2D();

   private void doSnapAndWiggle(FramePose3D solePoseToSnapAndWiggle,
                                ConvexPolygon2DReadOnly footStepPolygonInSoleFrame,
                                FrameConvexPolygon2DReadOnly footPolygonInWorld,
                                ConvexPolygon2DBasics snappedFootholdInWorldToPack)
   {
      if (!snapper.snapPolygonToPlanarRegionsList(footPolygonInWorld, steppableRegionsProvider.getSteppableRegions(), Double.POSITIVE_INFINITY, regionToSnapTo, snapTransform))
      {
         throw new RuntimeException("Snapping failed");
      }

      solePoseToSnapAndWiggle.setZ(0.0);
      solePoseToSnapAndWiggle.applyTransform(snapTransform);

      if (planarRegionSnapperCallback != null)
         planarRegionSnapperCallback.recordSnapTransform(snapTransform, regionToSnapTo);

      planarRegionFrame.setPoseAndUpdate(regionToSnapTo.getTransformToWorld());
      soleFrameAfterSnapAndBeforeWiggle.setPoseAndUpdate(solePoseToSnapAndWiggle);

      soleFrameAfterSnapAndBeforeWiggle.getTransformToDesiredFrame(transformFromSoleToRegion, planarRegionFrame);
      footPolygonInRegionFrame.set(footStepPolygonInSoleFrame);
      footPolygonInRegionFrame.applyTransform(transformFromSoleToRegion, false);

      stepConstraintOptimizer.reset();
      RigidBodyTransformReadOnly wiggleTransform = stepConstraintOptimizer.findConstraintTransform(footPolygonInRegionFrame, regionToSnapTo.getConvexHull(), wiggleParameters);

      if (wiggleTransform == null)
         solePoseToSnapAndWiggle.setToNaN();
      else
      {
         solePoseToSnapAndWiggle.changeFrame(planarRegionFrame);
         solePoseToSnapAndWiggle.applyTransform(wiggleTransform);
         solePoseToSnapAndWiggle.changeFrame(ReferenceFrame.getWorldFrame());

         if (planarRegionSnapperCallback != null)
            planarRegionSnapperCallback.recordWiggleTransform(wiggleTransform);
      }

      // check for partial foothold
      if (wiggleParameters.getDesiredDistanceInside() < 0.0)
      {

         soleFrameAfterWiggle.setPoseAndUpdate(solePoseToSnapAndWiggle);
         soleFrameAfterWiggle.getTransformToDesiredFrame(transformFromSoleToRegion, planarRegionFrame);
         footPolygonInRegionFrame.set(footStepPolygonInSoleFrame);
         footPolygonInRegionFrame.applyTransform(transformFromSoleToRegion, false);

         convexPolygonTools.computeIntersectionOfPolygons(regionToSnapTo.getConvexHull(), footPolygonInRegionFrame, snappedFootholdInWorldToPack);

         snappedFootholdInWorldToPack.applyInverseTransform(transformFromSoleToRegion, false);
      }
      else
      {
         snappedFootholdInWorldToPack.set(footPolygonInWorld);
      }
   }

   private final Point2D concaveHullVertex = new Point2D();
   private final ConvexPolygon2D boundaryOfModeledWorld = new ConvexPolygon2D();

   private boolean isFootPolygonOnBoundaryOfPlanarRegions(List<PlanarRegion> planarRegionsList, FrameConvexPolygon2D footPolygonInWorld)
   {
      boundaryOfModeledWorld.clear();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion region = planarRegionsList.get(i);

         for (int j = 0; j < region.getConcaveHullSize(); j++)
         {
            region.getTransformToWorld().transform(region.getConcaveHullVertex(j), concaveHullVertex, false);
            boundaryOfModeledWorld.addVertex(concaveHullVertex);
         }
      }
      boundaryOfModeledWorld.update();

      for (int i = 0; i < footPolygonInWorld.getNumberOfVertices(); i++)
      {
         if (!boundaryOfModeledWorld.isPointInside(footPolygonInWorld.getVertex(i)))
            return true;
      }
      return false;
   }

   public PlanarRegion findClosestPlanarRegionToPointByProjectionOntoXYPlane(double x, double y)
   {
      double shortestDistanceToPoint = Double.POSITIVE_INFINITY;
      PlanarRegion closestRegion = null;

      for (int i = 0; i < steppableRegionsProvider.getSteppableRegions().size(); i++)
      {
         PlanarRegion candidateRegion = steppableRegionsProvider.getSteppableRegions().get(i);
         double distanceToRegion = candidateRegion.distanceToPointByProjectionOntoXYPlane(x, y);
         if (distanceToRegion < shortestDistanceToPoint)
         {
            shortestDistanceToPoint = distanceToRegion;
            closestRegion = candidateRegion;
         }
      }

      return closestRegion;
   }
}
