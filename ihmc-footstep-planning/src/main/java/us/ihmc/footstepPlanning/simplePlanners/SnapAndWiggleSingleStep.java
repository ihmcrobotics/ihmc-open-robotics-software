package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class SnapAndWiggleSingleStep
{
   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final SnapAndWiggleSingleStepParameters parameters;
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public SnapAndWiggleSingleStep(SnapAndWiggleSingleStepParameters parameters)
   {
      this.parameters = parameters;
      wiggleParameters.deltaInside = parameters.getWiggleInsideDelta();
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      PlanarRegionsList steppableRegions = new PlanarRegionsList(planarRegionsList.getPlanarRegionsAsList()
                                                                                  .stream()
                                                                                  .filter(region -> region.getConvexHull().getArea() >= parameters.getMinPlanarRegionArea())
                                                                                  .filter(region -> region.getNormal().getZ() >= Math.cos(parameters.getMaxPlanarRegionAngle()))
                                                                                  .collect(Collectors.toList()));
      this.planarRegionsList.set(steppableRegions);
   }

   public ConvexPolygon2D snapAndWiggle(FramePose3D solePose, ConvexPolygon2DReadOnly footStepPolygon, boolean walkingForward) throws SnappingFailedException
   {
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return null;
      }

      FramePose3D solePoseBeforeSnapping = new FramePose3D(solePose);
      PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", solePose);
      FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D(soleFrameBeforeSnapping, footStepPolygon);
      footPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame()); // this works if the soleFrames are z up.

      if(isOnBoundaryOfPlanarRegions(planarRegionsList, footPolygon))
      {
         /*
          * If foot is on the boundary of planar regions, don't snap/wiggle but
          * set it to the nearest plane's height
          */
         FixedFramePoint3DBasics footPosition = solePose.getPosition();
         PlanarRegion closestRegion = planarRegionsList.findClosestPlanarRegionToPointByProjectionOntoXYPlane(footPosition.getX(), footPosition.getY());
         solePose.setZ(closestRegion.getPlaneZGivenXY(footPosition.getX(), footPosition.getY()));
         return new ConvexPolygon2D(footStepPolygon);
      }

      ConvexPolygon2D foothold = doSnapAndWiggle(solePose, footStepPolygon, footPolygon);
      checkAndHandleTopOfCliff(solePoseBeforeSnapping, solePose, walkingForward, footStepPolygon, footPolygon);
      checkAndHandleBottomOfCliff(solePose);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      solePose.get(soleTransform);
      foothold.applyInverseTransform(soleTransform, false);

      return foothold;
   }

   private ConvexPolygon2D doSnapAndWiggle(FramePose3D solePose, ConvexPolygon2DReadOnly footStepPolygon, FrameConvexPolygon2D footPolygon)
         throws SnappingFailedException
   {
      PlanarRegion regionToMoveTo = new PlanarRegion();
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, regionToMoveTo);
      if (snapTransform == null)
      {
         throw new SnappingFailedException();
      }

      solePose.setZ(0.0);
      solePose.applyTransform(snapTransform);

      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      regionToMoveTo.getTransformToWorld(regionToWorld);
      PoseReferenceFrame regionFrame = new PoseReferenceFrame("RegionFrame", ReferenceFrame.getWorldFrame());
      regionFrame.setPoseAndUpdate(regionToWorld);
      PoseReferenceFrame soleFrameBeforeWiggle = new PoseReferenceFrame("SoleFrameBeforeWiggle", solePose);

      RigidBodyTransform soleToRegion = soleFrameBeforeWiggle.getTransformToDesiredFrame(regionFrame);
      ConvexPolygon2D footPolygonInRegion = new ConvexPolygon2D(footStepPolygon);
      footPolygonInRegion.applyTransform(soleToRegion, false);

      RigidBodyTransform wiggleTransform = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegion, regionToMoveTo, wiggleParameters);

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
         convexPolygonTools.computeIntersectionOfPolygons(regionToMoveTo.getConvexHull(), footPolygonInRegion, foothold);
         soleToRegion.invert();
         foothold.applyTransform(soleToRegion, false);
      }
      else
      {
         foothold.set(footPolygon);
      }
      return foothold;
   }

   private boolean isOnBoundaryOfPlanarRegions(PlanarRegionsList planarRegionsList, FrameConvexPolygon2D footPolygonInWorld)
   {
      PoseReferenceFrame planarRegionFrame = new PoseReferenceFrame("PlanarRegionFrame", ReferenceFrame.getWorldFrame());
      FrameConvexPolygon2D planarRegionPolygon = new FrameConvexPolygon2D();
      ConvexPolygon2D planarRegionsBoundingPolygon = new ConvexPolygon2D();

      planarRegionsBoundingPolygon.clear();
      for(PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         region.getTransformToWorld(transform);
         planarRegionFrame.setPoseAndUpdate(transform);

         planarRegionPolygon.set(region.getConvexHull());
         planarRegionPolygon.setReferenceFrame(planarRegionFrame);
         planarRegionPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

         planarRegionsBoundingPolygon.addVertices(planarRegionPolygon);
      }
      planarRegionsBoundingPolygon.update();

      for (int i = 0; i < footPolygonInWorld.getNumberOfVertices(); i++)
      {
         if (!planarRegionsBoundingPolygon.isPointInside(footPolygonInWorld.getVertex(i)))
         {
            return true;
         }
      }
      return false;
   }

   // method to avoid getting stuck at the top of a ledge
   private void checkAndHandleTopOfCliff(FramePose3D inputPose, FramePose3D outputPose, boolean walkingForward, ConvexPolygon2DReadOnly footStepPolygon, FrameConvexPolygon2D footPolygon)
   {
      if(Double.isNaN(parameters.getFootLength()) || parameters.getWiggleInWrongDirectionThreshold() <= 0.0)
      {
         return;
      }

      double yaw = inputPose.getYaw();
      double desiredForwardHeading = walkingForward ? 1.0 : -1.0;
      Vector2D desiredHeading = new Vector2D(Math.cos(yaw) * desiredForwardHeading, Math.sin(yaw) * desiredForwardHeading);
      Vector2D achievedHeading = new Vector2D(outputPose.getX() - inputPose.getX(), outputPose.getY() - inputPose.getY());
      double projectionScale = achievedHeading.dot(desiredHeading);

      if(projectionScale < - parameters.getWiggleInWrongDirectionThreshold())
      {
         FramePose3D shiftedPose = new FramePose3D(inputPose);
         desiredHeading.scale(parameters.getFootLength() + wiggleParameters.deltaInside);
         shiftedPose.prependTranslation(desiredHeading.getX(), desiredHeading.getY(), 0.0);
         outputPose.set(shiftedPose);
         try
         {
            doSnapAndWiggle(outputPose, footStepPolygon, footPolygon);
         }
         catch(SnappingFailedException e)
         {
         }
      }
   }

   private void checkAndHandleBottomOfCliff(FramePose3D footPose)
   {
      if (Double.isNaN(parameters.getFootLength()) || parameters.getCliffHeightToAvoid() <= 0.0 || parameters.getClosestDistanceToCliff() <= 0.0)
      {
         return;
      }

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      footPose.get(soleTransform);

      ArrayList<LineSegment2D> lineSegmentsInSoleFrame = new ArrayList<>();
      lineSegmentsInSoleFrame
            .add(new LineSegment2D(0.5 * parameters.getFootLength(), 0.0, 0.5 * parameters.getFootLength() + parameters.getClosestDistanceToCliff(), 0.0));
      lineSegmentsInSoleFrame
            .add(new LineSegment2D(-0.5 * parameters.getFootLength(), 0.0, -0.5 * parameters.getFootLength() - parameters.getClosestDistanceToCliff(), 0.0));

      Point3D highestPointInSoleFrame = new Point3D();
      LineSegment2D highestLineSegmentInSoleFrame = new LineSegment2D();
      Point3D closestPointOnCliff = new Point3D();
      double highestPointZ = PlanarRegionBaseOfCliffAvoider
            .findHighestPointInFrame(planarRegionsList.get(), soleTransform, lineSegmentsInSoleFrame, highestPointInSoleFrame, highestLineSegmentInSoleFrame,
                                     closestPointOnCliff);

      if (highestPointZ > parameters.getCliffHeightToAvoid())
      {
         double shiftSign = Math.signum(-closestPointOnCliff.getX());
         double shiftAmount = shiftSign * (parameters.getClosestDistanceToCliff() - (Math.abs(closestPointOnCliff.getX()) - 0.5 * parameters.getFootLength()));

         double footstepYaw = footPose.getYaw();
         footPose.prependTranslation(shiftAmount * Math.cos(footstepYaw), shiftAmount * Math.sin(footstepYaw), 0.0);
      }
   }

   public static class SnappingFailedException extends Exception
   {
      private static final long serialVersionUID = 6962526781987562540L;

      private SnappingFailedException()
      {
         super("Foot Snapping Failed");
      }
   }
}
