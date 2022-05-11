package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStep;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class PlanarRegionFootstepSnapper implements FootstepAdjustment
{
   private final RecyclingArrayList<PlanarRegion> steppableRegionsList = new RecyclingArrayList<>(PlanarRegion::new);
   private final SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();
   private final WiggleParameters wiggleParameters = new WiggleParameters();

   private final FramePose3D footstepAtSameHeightAsStanceFoot = new FramePose3D();
   private final FramePose3D adjustedFootstepPose = new FramePose3D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();

   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final ContinuousStepGenerator continuousStepGenerator;

   public PlanarRegionFootstepSnapper(ContinuousStepGenerator continuousStepGenerator, SteppingParameters steppingParameters)
   {
      this.continuousStepGenerator = continuousStepGenerator;

      double footLength = steppingParameters.getFootLength();
      double toeWidth = steppingParameters.getToeWidth();
      double footWidth = steppingParameters.getFootWidth();
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.update();
      footPolygons = new SideDependentList<>(footPolygon, footPolygon);
   }


   public void setPlanarRegions(PlanarRegionsListCommand planarRegions)
   {
      steppableRegionsList.clear();
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         if (EuclidGeometryPolygonTools.computeConvexPolygon2DArea(candidateRegion.getConcaveHullsVertices(),
                                                                   candidateRegion.getConcaveHullsVertices().size(),
                                                                   true,
                                                                   null) > parameters.getMinPlanarRegionArea())
            continue;

         if (candidateRegion.getTransformToWorld().getM22() >= Math.cos(parameters.getMaxPlanarRegionAngle()))
            continue;

         PlanarRegion planarRegion = steppableRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getRegionId());
      }
   }

   @Override
   public FramePose3DReadOnly adjustFootstep(FramePose2DReadOnly footstepPose, RobotSide footSide)
   {
      footstepAtSameHeightAsStanceFoot.getPosition().set(footstepPose.getPosition());
      footstepAtSameHeightAsStanceFoot.setZ(continuousStepGenerator.getCurrentSupportFootPose().getZ());
      footstepAtSameHeightAsStanceFoot.getOrientation().set(footstepPose.getOrientation());

      if (steppableRegionsList.isEmpty())
      {
         adjustedFootstepPose.set(footstepAtSameHeightAsStanceFoot);
         ConvexPolygon2D footPolygonToWiggle = new ConvexPolygon2D();
         ConvexPolygon2D wiggledPolygon = new ConvexPolygon2D();
         footPolygonToWiggle.set(footPolygons.get(footSide));
         try
         {
            snapAndWiggle(adjustedFootstepPose, footPolygonToWiggle, wiggledPolygon);
            if (adjustedFootstepPose.containsNaN())
               return footstepAtSameHeightAsStanceFoot;
         }
         catch (RuntimeException e)
         {
            /*
             * It's fine if the snap & wiggle fails, can be because there are no planar regions around the footstep.
             * Let's just keep the adjusted footstep based on the pose of the current stance foot.
             */
         }
         return adjustedFootstepPose;
      }
      else
      {
         return footstepAtSameHeightAsStanceFoot;
      }
   }

   private final PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", ReferenceFrame.getWorldFrame());

   private final FrameConvexPolygon2D footstepPolygonInWorld = new FrameConvexPolygon2D();

   public boolean snapAndWiggle(FramePose3D solePose, ConvexPolygon2DReadOnly footStepPolygonInSoleFrame, ConvexPolygon2DBasics snappedFootstepPolygonToPack)
   {
      if (steppableRegionsList.isEmpty())
      {
         snappedFootstepPolygonToPack.clear();
         return false;
      }

      soleFrameBeforeSnapping.setPoseAndUpdate(solePose);
      footstepPolygonInWorld.setIncludingFrame(soleFrameBeforeSnapping, footStepPolygonInSoleFrame);
      footstepPolygonInWorld.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame()); // this works if the soleFrames are z up.

      if (isFootPolygonOnBoundaryOfPlanarRegions(steppableRegionsList, footstepPolygonInWorld))
      {
         /*
          * If foot is on the boundary of planar regions, don't snap/wiggle but
          * set it to the nearest plane's height
          */
         FixedFramePoint3DBasics footPosition = solePose.getPosition();
         PlanarRegion closestRegion = findClosestPlanarRegionToPointByProjectionOntoXYPlane(footPosition.getX(), footPosition.getY());
         footPosition.setZ(closestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
         snappedFootstepPolygonToPack.set(footStepPolygonInSoleFrame);
         return true;
      }

      // TODO garbasge
      snappedFootstepPolygonToPack.set(doSnapAndWiggle(solePose, footStepPolygonInSoleFrame, footstepPolygonInWorld));
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      solePose.get(soleTransform);
      snappedFootstepPolygonToPack.applyInverseTransform(soleTransform, false);

      return true;
   }

   private final PlanarRegion regionToSnapTo = new PlanarRegion();
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();

   private ConvexPolygon2D doSnapAndWiggle(FramePose3D solePose, ConvexPolygon2DReadOnly footStepPolygon, FrameConvexPolygon2D footPolygon)
   {
      if (!snapper.snapPolygonToPlanarRegionsList(footPolygon, steppableRegionsList, Double.POSITIVE_INFINITY, regionToSnapTo, snapTransform))
      {
         throw new RuntimeException("Snapping failed");
      }

      solePose.setZ(0.0);
      solePose.applyTransform(snapTransform);

      // TODO remove garbage
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      regionToSnapTo.getTransformToWorld(regionToWorld);
      PoseReferenceFrame regionFrame = new PoseReferenceFrame("RegionFrame", ReferenceFrame.getWorldFrame());
      regionFrame.setPoseAndUpdate(regionToWorld);
      PoseReferenceFrame soleFrameBeforeWiggle = new PoseReferenceFrame("SoleFrameBeforeWiggle", solePose);

      RigidBodyTransform soleToRegion = soleFrameBeforeWiggle.getTransformToDesiredFrame(regionFrame);
      ConvexPolygon2D footPolygonInRegion = new ConvexPolygon2D(footStepPolygon);
      footPolygonInRegion.applyTransform(soleToRegion, false);

      RigidBodyTransform wiggleTransform = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegion, regionToSnapTo, wiggleParameters);

      if (wiggleTransform == null)
         solePose.setToNaN();
      else
      {
         solePose.changeFrame(regionFrame);
         solePose.applyTransform(wiggleTransform);
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // check for partial foothold
      ConvexPolygon2D foothold = new ConvexPolygon2D();
      if (wiggleParameters.deltaInside < 0.0)
      {
         PoseReferenceFrame soleFrameAfterWiggle = new PoseReferenceFrame("SoleFrameAfterWiggle", solePose);
         soleToRegion = soleFrameAfterWiggle.getTransformToDesiredFrame(regionFrame);
         footPolygonInRegion.set(footStepPolygon);
         footPolygonInRegion.applyTransform(soleToRegion, false);
         convexPolygonTools.computeIntersectionOfPolygons(regionToSnapTo.getConvexHull(), footPolygonInRegion, foothold);
         soleToRegion.invert();
         foothold.applyTransform(soleToRegion, false);
      }
      else
      {
         foothold.set(footPolygon);
      }
      return foothold;
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

      for (int i = 0; i < steppableRegionsList.size(); i++)
      {
         PlanarRegion candidateRegion = steppableRegionsList.get(i);
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
