package us.ihmc.humanoidBehaviors.taskExecutor;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LimbName;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FootstepTask extends BehaviorTask
{
   private final FootstepListBehavior footstepListBehavior;
   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

   public FootstepTask(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, FootstepListBehavior footstepListBehavior, FramePose footPose, DoubleYoVariable yoTime)
   {
      super(footstepListBehavior, yoTime);
      ReferenceFrame soleFrame;
      RigidBody endEffector;
      
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      endEffector = fullRobotModel.getEndEffector(robotSide, LimbName.LEG);
      
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", ReferenceFrame.getWorldFrame());
      poseReferenceFrame.setPoseAndUpdate(footPose);
      footsteps.add(new Footstep(endEffector, robotSide, soleFrame, poseReferenceFrame));
      this.footstepListBehavior = footstepListBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      footstepListBehavior.set(footsteps);      
   }
   
   @Override
   public void doTransitionOutOfAction()
   {
      footstepListBehavior.doPostBehaviorCleanup();
      footsteps.clear();
   }
}
