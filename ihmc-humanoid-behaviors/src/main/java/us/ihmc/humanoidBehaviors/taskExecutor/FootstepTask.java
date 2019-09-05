package us.ihmc.humanoidBehaviors.taskExecutor;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepTask extends BehaviorAction
{
   private final FootstepListBehavior footstepListBehavior;
   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

   public FootstepTask(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, FootstepListBehavior footstepListBehavior, FramePose3D footPose)
   {
      super(footstepListBehavior);
      footsteps.add(new Footstep(robotSide, footPose));
      this.footstepListBehavior = footstepListBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      footstepListBehavior.set(footsteps);
   }

   @Override
   public void onExit()
   {
      footstepListBehavior.doPostBehaviorCleanup();
      footsteps.clear();
   }
}
