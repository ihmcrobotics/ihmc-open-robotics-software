package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootTrajectoryTask extends BehaviorAction
{
   private final FootTrajectoryMessage footTrajectoryMessage;
   private final FootTrajectoryBehavior footPoseBehavior;

   public FootTrajectoryTask(RobotSide robotSide, Point3D position, Quaternion orientation, FootTrajectoryBehavior behavior, double trajectoryTime)
   {

      super(behavior);
      this.footPoseBehavior = behavior;
      footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   public FootTrajectoryTask(RobotSide robotSide, FramePose3D pose, FootTrajectoryBehavior behavior, double trajectoryTime)
   {
      super(behavior);
      this.footPoseBehavior = behavior;

      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      pose.get(position, orientation);
      footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   @Override
   protected void setBehaviorInput()
   {
      footPoseBehavior.setInput(footTrajectoryMessage);
   }
}
