package us.ihmc.humanoidBehaviors.behaviors.goalLocation;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ThreadedBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;

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

   public abstract void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack);
}
