package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ThreadedBehaviorService;
import us.ihmc.ros2.ROS2Node;

public abstract class GoalDetectorBehaviorService extends ThreadedBehaviorService
{
   public GoalDetectorBehaviorService(String robotName, String threadName, ROS2Node ros2Node)
   {
      super(robotName, threadName, ros2Node);
   }

   public abstract boolean getGoalHasBeenLocated();

   public abstract void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack);
}
