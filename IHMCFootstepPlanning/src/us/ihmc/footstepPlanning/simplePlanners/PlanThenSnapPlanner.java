package us.ihmc.footstepPlanning.simplePlanners;

import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
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
   private RobotSide stepSide;
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
      stepSide = stanceSide.getOppositeSide();
   }

   @Override
   public void setGoalPose(FramePose goalPose)
   {
      internalPlanner.setGoalPose(goalPose);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan(List<FramePose> solePosesToPack)
   {
      FootstepPlanningResult result = internalPlanner.plan(solePosesToPack);
      if (planarRegionsList == null)
         return result;

      for (FramePose solePose : solePosesToPack)
      {
         PoseReferenceFrame soleFrameBeforeSnapping = new PoseReferenceFrame("SoleFrameBeforeSnapping", solePose);
         FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d(soleFrameBeforeSnapping, footPolygons.get(stepSide));
         footPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         RigidBodyTransform snapTransform =
               PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon.getConvexPolygon2d(), planarRegionsList);
         if (snapTransform == null)
            return FootstepPlanningResult.SNAPPING_FAILED;
         solePose.applyTransform(snapTransform);
         stepSide = stepSide.getOppositeSide();
      }
      return result;
   }

}
