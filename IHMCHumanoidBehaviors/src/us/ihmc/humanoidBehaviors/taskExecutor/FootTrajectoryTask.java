package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final FootTrajectoryMessage footTrajectoryMessage;
   private final FootTrajectoryBehavior footPoseBehavior;

   
   
   public FootTrajectoryTask(RobotSide robotSide, Point3D position, Quaternion orientation, FootTrajectoryBehavior behavior,
         double trajectoryTime)
   {
      this(null, robotSide,  position,  orientation,  behavior,   trajectoryTime);
   }
   
   public FootTrajectoryTask(E stateEnum,RobotSide robotSide, Point3D position, Quaternion orientation, FootTrajectoryBehavior behavior,
         double trajectoryTime)
   {
     
      super(stateEnum,behavior);
      this.footPoseBehavior = behavior;
      footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   public FootTrajectoryTask(RobotSide robotSide, FramePose pose, FootTrajectoryBehavior behavior, double trajectoryTime)
   {
      this(null, robotSide,  pose,  behavior,  trajectoryTime);
   }

   public FootTrajectoryTask(E stateEnum,RobotSide robotSide, FramePose pose, FootTrajectoryBehavior behavior, double trajectoryTime)
   {
      super(stateEnum,behavior);
      this.footPoseBehavior = behavior;

      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      pose.getPose(position, orientation);   
      footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, position, orientation);
   }

   @Override
   protected void setBehaviorInput()
   {
      footPoseBehavior.setInput(footTrajectoryMessage);
   }

}
