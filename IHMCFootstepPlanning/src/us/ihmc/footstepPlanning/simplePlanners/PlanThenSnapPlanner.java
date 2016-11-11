package us.ihmc.footstepPlanning.simplePlanners;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
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
         footPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         Pair<RigidBodyTransform, PlanarRegion> snapTransformAndRegion = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon.getConvexPolygon2d(), planarRegionsList);
         if (snapTransformAndRegion == null)
            return FootstepPlanningResult.SNAPPING_FAILED;

         solePose.applyTransform(snapTransformAndRegion.getLeft());
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
