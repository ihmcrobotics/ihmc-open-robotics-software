package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootTrajectoryTask extends BehaviorTask
{
   private final FootTrajectoryMessage footTrajectoryMessage;
   private final FootTrajectoryBehavior footPoseBehavior;

   public FootTrajectoryTask(RobotSide robotSide, Point3d position, Quat4d orientation, DoubleYoVariable yoTime, FootTrajectoryBehavior behavior,
         double trajectoryTime)
   {
     
      super(behavior, yoTime);
      this.footPoseBehavior = behavior;
      footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   public FootTrajectoryTask(RobotSide robotSide, FramePose pose, DoubleYoVariable yoTime, FootTrajectoryBehavior behavior, double trajectoryTime)
   {
      super(behavior, yoTime);
      this.footPoseBehavior = behavior;

      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();
      pose.getPose(position, orientation);
      footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   @Override
   protected void setBehaviorInput()
   {
      footPoseBehavior.setInput(footTrajectoryMessage);
   }

}
