package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FollowFiducialBehavior.FollowFiducialBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;

public class FollowFiducialBehavior extends StateMachineBehavior<FollowFiducialBehaviorStates>
{
   public static final int DEFAULT_FIDUCIAL_TO_FOLLOW = 50;

   public enum FollowFiducialBehaviorStates
   {
      LOCATE_FIDUCIAL, WALK_TO_FIDUCIAL,
   }

   private final FramePose latestFiducialPoseWorld = new FramePose();

   private final WalkToFiducialBehavior walkToFiducialBehavior;

   public FollowFiducialBehavior(DoubleYoVariable yoTime, CommunicationBridge communicationBridge)
   {
      super(FollowFiducialBehavior.class.getSimpleName(), FollowFiducialBehaviorStates.class, yoTime, communicationBridge);

      walkToFiducialBehavior = new WalkToFiducialBehavior(communicationBridge);
      setupStateMachine();
   }
   
   public void setTargetFiducialId(int targetFiducialId)
   {
//      locateFiducialBehavior.setTargetIDToLocate(targetFiducialId);
   }

   private void setupStateMachine()
   {

      BehaviorAction<FollowFiducialBehaviorStates> walkToFiducialBehaviorAction = new BehaviorAction<FollowFiducialBehaviorStates>(FollowFiducialBehaviorStates.WALK_TO_FIDUCIAL,
                                                                                                                                   walkToFiducialBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
//            locateFiducialBehavior.getFiducialPoseWorldFrame().getFramePoseIncludingFrame(latestFiducialPoseWorld);
            walkToFiducialBehavior.setGoalPose(latestFiducialPoseWorld);

            TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("Walking to fiducial");
            sendPacket(textToSpeechPacket);
         }
      };

      // for now just alternate between locating and walking:
//      statemachine.addStateWithDoneTransition(locateFiducialBehaviorAction, FollowFiducialBehaviorStates.WALK_TO_FIDUCIAL);
      statemachine.addStateWithDoneTransition(walkToFiducialBehaviorAction, FollowFiducialBehaviorStates.LOCATE_FIDUCIAL);

//      statemachine.setCurrentState(FollowFiducialBehaviorStates.LOCATE_FIDUCIAL);
   }
}
