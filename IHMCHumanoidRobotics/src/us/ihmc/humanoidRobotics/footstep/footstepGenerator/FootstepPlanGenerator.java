package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose2d;
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

   private RobotSide initialStanceSide;

   public FootstepPlanGenerator(FootstepPlanner planner, SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames)
   {
      this.planner = planner;
      this.feet = feet;
      this.soleFrames = soleFrames;
   }

   @Override
   public List<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> footsteps = new ArrayList<>();
      List<FramePose2d> footstepPoses = plan();
      RobotSide stepSide = initialStanceSide.getOppositeSide();

      for (FramePose2d footstepPose : footstepPoses)
      {
         RigidBodyTransform footstepTransform = new RigidBodyTransform();
         footstepTransform.setRotationYawAndZeroTranslation(footstepPose.getYaw());
         footstepTransform.setTranslation(footstepPose.getX(), footstepPose.getY(), 0.0);

         PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("FootstepFrame", ReferenceFrame.getWorldFrame());
         footstepPoseFrame.setPoseAndUpdate(footstepTransform);
         footsteps.add(new Footstep(feet.get(stepSide), stepSide, soleFrames.get(stepSide), footstepPoseFrame));
      }

      return footsteps;
   }

   @Override
   public void setInitialStanceFoot(FramePose2d stanceFootPose, RobotSide side)
   {
      initialStanceSide = side;
      planner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoalPose(FramePose2d goalPose)
   {
      planner.setGoalPose(goalPose);
   }

   @Override
   public List<FramePose2d> plan()
   {
      return planner.plan();
   }

}
