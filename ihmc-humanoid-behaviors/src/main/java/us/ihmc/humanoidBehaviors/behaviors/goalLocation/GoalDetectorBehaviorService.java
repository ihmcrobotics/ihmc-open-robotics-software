package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ThreadedBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.geometry.FramePose;

/**
 *
 */
public abstract class GoalDetectorBehaviorService extends ThreadedBehaviorService
{
   public GoalDetectorBehaviorService(String threadName, CommunicationBridgeInterface communicationBridge)
   {
      super(threadName, communicationBridge);
   }

   public abstract boolean getGoalHasBeenLocated();

   public abstract void getReportedGoalPoseWorldFrame(FramePose framePoseToPack);
}
