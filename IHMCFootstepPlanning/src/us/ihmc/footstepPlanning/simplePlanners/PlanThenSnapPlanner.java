package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.footstepPlanning.polygonWiggling.PolygonWiggler;
import us.ihmc.footstepPlanning.polygonWiggling.WiggleParameters;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class PlanThenSnapPlanner implements FootstepPlanner
{
   private final WiggleParameters wiggleParameters = new WiggleParameters();

   private final FootstepPlanner internalPlanner;
   private final SideDependentList<ConvexPolygon2d> footPolygons;
   private PlanarRegionsList planarRegionsList;

   public PlanThenSnapPlanner(FootstepPlanner internalPlanner, SideDependentList<ConvexPolygon2d> footPolygons)
   {
      this.internalPlanner = internalPlanner;
      this.footPolygons = footPolygons;
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide stanceSide)
   {
      internalPlanner.setInitialStanceFoot(stanceFootPose, stanceSide);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      internalPlanner.setGoal(goal);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   private FootstepPlan footstepPlan = new FootstepPlan();

   @Override
   public FootstepPlanningResult plan()
   {
      FootstepPlanningResult result = internalPlanner.plan();
      footstepPlan = internalPlanner.getPlan();

      if (planarRegionsList == null)
         return result;

      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      for (int i=0; i<numberOfFootsteps; i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose solePose = new FramePose();
         footstep.getSoleFramePose(solePose);

         PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", solePose);
         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(soleFrameBeforeSnapping, footPolygons.get(footstep.getRobotSide()));
         footPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame()); // this works if the soleFrames are z up.

         PlanarRegion regionToMoveTo = new PlanarRegion();
         RigidBodyTransform snapTransform =
               PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon.getConvexPolygon2d(), planarRegionsList, regionToMoveTo);
         if (snapTransform == null)
            return FootstepPlanningResult.SNAPPING_FAILED;
         solePose.setZ(0.0);
         solePose.applyTransform(snapTransform);

         RigidBodyTransform regionToWorld = new RigidBodyTransform();
         regionToMoveTo.getTransformToWorld(regionToWorld);
         PoseReferenceFrame regionFrame = new PoseReferenceFrame("RegionFrame", ReferenceFrame.getWorldFrame());
         regionFrame.setPoseAndUpdate(regionToWorld);
         PoseReferenceFrame soleFrameBeforeWiggle = new PoseReferenceFrame("SoleFrameBeforeWiggle", solePose);

         RigidBodyTransform soleToRegion = soleFrameBeforeWiggle.getTransformToDesiredFrame(regionFrame);
         ConvexPolygon2d footPolygonInRegion = new ConvexPolygon2d(footPolygons.get(footstep.getRobotSide()));
         footPolygonInRegion.applyTransformAndProjectToXYPlane(soleToRegion);

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
            footPolygonInRegion.setAndUpdate(footPolygons.get(footstep.getRobotSide()));
            footPolygonInRegion.applyTransformAndProjectToXYPlane(soleToRegion);
            ConvexPolygon2d foothold = regionToMoveTo.getConvexHull().intersectionWith(footPolygonInRegion);
            soleToRegion.invert();
            foothold.applyTransformAndProjectToXYPlane(soleToRegion);
            footstep.setFoothold(foothold);
         }

         footstep.setSoleFramePose(solePose);
      }
      return result;
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlan;
   }

}
