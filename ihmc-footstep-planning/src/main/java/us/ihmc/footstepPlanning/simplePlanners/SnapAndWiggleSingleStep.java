package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class SnapAndWiggleSingleStep
{
   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private PlanarRegionsList planarRegionsList;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public ConvexPolygon2D snapAndWiggle(FramePose3D solePose, ConvexPolygon2D footStepPolygon) throws SnappingFailedException
   {

      if (planarRegionsList == null)
      {
         return null;
      }

      PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", solePose);
      FrameConvexPolygon2D footPolygon = new FrameConvexPolygon2D(soleFrameBeforeSnapping, footStepPolygon);
      footPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame()); // this works if the soleFrames are z up.

      PlanarRegion regionToMoveTo = new PlanarRegion();
      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList,
            regionToMoveTo);
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

      // TODO: make the delta inside value part of the optimization.
      wiggleParameters.deltaInside = 0.0;
      RigidBodyTransform wiggleTransform = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegion, regionToMoveTo, wiggleParameters);
      if (wiggleTransform == null)
      {
         wiggleParameters.deltaInside = -0.055;
         wiggleTransform = PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegion, regionToMoveTo, wiggleParameters);
      }

      if (wiggleTransform == null)
         solePose.setToNaN();
      else
      {
         solePose.changeFrame(regionFrame);
         solePose.applyTransform(wiggleTransform);
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      // fix the foothold
      if (wiggleParameters.deltaInside != 0.0)
      {
         PoseReferenceFrame soleFrameAfterWiggle = new PoseReferenceFrame("SoleFrameAfterWiggle", solePose);
         soleToRegion = soleFrameAfterWiggle.getTransformToDesiredFrame(regionFrame);
         footPolygonInRegion.set(footStepPolygon);
         footPolygonInRegion.applyTransform(soleToRegion, false);
         ConvexPolygon2D foothold = new ConvexPolygon2D();
         convexPolygonTools.computeIntersectionOfPolygons(regionToMoveTo.getConvexHull(), footPolygonInRegion, foothold);
         soleToRegion.invert();
         foothold.applyTransform(soleToRegion, false);
         return foothold;
      }
      return null;
   }

   private class SnappingFailedException extends Exception
   {
      private SnappingFailedException()
      {
         super("Foot Snapping_Failed");
      }
   }
   
   public static void main(String[] args)
   {
      PoseReferenceFrame bot1 = new PoseReferenceFrame("bot1", ReferenceFrame.getWorldFrame());
      bot1.setPositionWithoutChecksAndUpdate(0, 2, 0);
      
      PoseReferenceFrame bot2 = new PoseReferenceFrame("bot2", ReferenceFrame.getWorldFrame());
      bot2.setPositionWithoutChecksAndUpdate(0, -3, 0);
      
      
      bot1.update();
      bot2.update();
      RigidBodyTransform transform = bot1.getTransformToDesiredFrame(bot2);
      
      System.out.println(transform);
   }
}
