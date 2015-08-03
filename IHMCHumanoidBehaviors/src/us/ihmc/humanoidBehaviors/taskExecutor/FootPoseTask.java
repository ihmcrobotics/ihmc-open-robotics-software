package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class FootPoseTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final FootPosePacket footPosePacket;
   private final FootPoseBehavior footPoseBehavior;

   public FootPoseTask(RobotSide robotSide, Point3d position, Quat4d orientation, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior,
         double trajectoryTime)
   {
      this(robotSide, position, orientation, yoTime, footPoseBehavior, trajectoryTime, 0.0);
   }

   public FootPoseTask(RobotSide robotSide, Point3d position, Quat4d orientation, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior,
         double trajectoryTime, double sleepTime)
   {
      super(footPoseBehavior, yoTime, sleepTime);
      this.footPoseBehavior = footPoseBehavior;
      footPosePacket = new FootPosePacket(robotSide, position, orientation, trajectoryTime);
   }

   public FootPoseTask(RobotSide robotSide, FramePose pose, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior, double trajectoryTime, double sleepTime)
   {
      super(footPoseBehavior, yoTime, sleepTime);
      this.footPoseBehavior = footPoseBehavior;

      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();
      pose.getPose(position, orientation);
      footPosePacket = new FootPosePacket(robotSide, position, orientation, trajectoryTime);
   }

   @Override
   protected void setBehaviorInput()
   {
      footPoseBehavior.setInput(footPosePacket);
   }

}
