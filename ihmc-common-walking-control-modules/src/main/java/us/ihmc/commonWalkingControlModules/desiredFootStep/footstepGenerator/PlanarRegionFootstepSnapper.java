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
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
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

         double polygonArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(candidateRegion.getConcaveHullsVertices(),
                                                                                         candidateRegion.getConcaveHullsVertices().size(),
                                                                                         true,
                                                                                         null);
         if (polygonArea > parameters.getMinPlanarRegionArea())
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
   private final RigidBodyTransform transformToSole = new RigidBodyTransform();

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
      doSnapAndWiggle(solePose, footStepPolygonInSoleFrame, footstepPolygonInWorld, snappedFootstepPolygonToPack);

      solePose.get(transformToSole);
      snappedFootstepPolygonToPack.applyInverseTransform(transformToSole, false);

      return true;
   }

   private final PlanarRegion regionToSnapTo = new PlanarRegion();
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
      if (!snapper.snapPolygonToPlanarRegionsList(footPolygonInWorld, steppableRegionsList, Double.POSITIVE_INFINITY, regionToSnapTo, snapTransform))
      {
         throw new RuntimeException("Snapping failed");
      }

      solePoseToSnapAndWiggle.setZ(0.0);
      solePoseToSnapAndWiggle.applyTransform(snapTransform);

      planarRegionFrame.setPoseAndUpdate(regionToSnapTo.getTransformToWorld());
      soleFrameAfterSnapAndBeforeWiggle.setPoseAndUpdate(solePoseToSnapAndWiggle);

      soleFrameAfterSnapAndBeforeWiggle.getTransformToDesiredFrame(transformFromSoleToRegion, planarRegionFrame);
      footPolygonInRegionFrame.set(footStepPolygonInSoleFrame);
      footPolygonInRegionFrame.applyTransform(transformFromSoleToRegion, false);

      // TODO remove garbage
      RigidBodyTransform wiggleTransform = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegionFrame, regionToSnapTo, wiggleParameters);

      if (wiggleTransform == null)
         solePoseToSnapAndWiggle.setToNaN();
      else
      {
         solePoseToSnapAndWiggle.changeFrame(planarRegionFrame);
         solePoseToSnapAndWiggle.applyTransform(wiggleTransform);
         solePoseToSnapAndWiggle.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // check for partial foothold
      if (wiggleParameters.deltaInside < 0.0)
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
