package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FootstepPlanGenerator implements FootstepGenerator, FootstepPlanner
{
   private final FootstepPlanner planner;
   private final SideDependentList<RigidBody> feet;
   private final SideDependentList<ReferenceFrame> soleFrames;
   private final SideDependentList<RigidBodyTransform> transformsSoleToAnkle = new SideDependentList<>();

   private RobotSide initialStanceSide;

   public FootstepPlanGenerator(FootstepPlanner planner, SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames)
   {
      this.planner = planner;
      this.feet = feet;
      this.soleFrames = soleFrames;

      for (RobotSide side : RobotSide.values)
      {
         RigidBody foot = feet.get(side);
         ReferenceFrame soleFrame = soleFrames.get(side);
         RigidBodyTransform transform = new RigidBodyTransform();
         foot.getParentJoint().getFrameAfterJoint().getTransformToDesiredFrame(transform, soleFrame);
         transformsSoleToAnkle.put(side, transform);
      }
   }

   @Override
   public List<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> footsteps = new ArrayList<>();
      List<FramePose> footstepPoses = plan();
      RobotSide stepSide = initialStanceSide.getOppositeSide();

      for (FramePose footstepPose : footstepPoses)
      {
         footstepPose.applyTransform(transformsSoleToAnkle.get(stepSide));
         PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("FootstepFrame", footstepPose);
         footsteps.add(new Footstep(feet.get(stepSide), stepSide, soleFrames.get(stepSide), footstepPoseFrame));
         stepSide = stepSide.getOppositeSide();
      }

      return footsteps;
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      initialStanceSide = side;
      planner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoalPose(FramePose goalPose)
   {
      planner.setGoalPose(goalPose);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planner.setPlanarRegions(planarRegionsList);
   }

   @Override
   public List<FramePose> plan()
   {
      return planner.plan();
   }

}
