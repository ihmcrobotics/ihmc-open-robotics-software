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
   public enum FollowFiducialBehaviorStates
   {
      LOCATE_FIDUCIAL, WALK_TO_FIDUCIAL,
   }

   private final FramePose latestFiducialPose = new FramePose();

   private final LocateFiducialBehavior locateFiducialBehavior;
   private final WalkToFiducialBehavior walkToFiducialBehavior;

   public FollowFiducialBehavior(DoubleYoVariable yoTime, CommunicationBridge communicationBridge)
   {
      super(FollowFiducialBehavior.class.getSimpleName(), FollowFiducialBehaviorStates.class, yoTime, communicationBridge);

      locateFiducialBehavior = new LocateFiducialBehavior(communicationBridge);
      walkToFiducialBehavior = new WalkToFiducialBehavior(communicationBridge);
      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<FollowFiducialBehaviorStates> locateFiducialBehaviorAction = new BehaviorAction<FollowFiducialBehaviorStates>(FollowFiducialBehaviorStates.LOCATE_FIDUCIAL,
                                                                                                                                   this.locateFiducialBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("Locating fiducial");
            sendPacket(textToSpeechPacket);
         }
      };

      BehaviorAction<FollowFiducialBehaviorStates> walkToFiducialBehaviorAction = new BehaviorAction<FollowFiducialBehaviorStates>(FollowFiducialBehaviorStates.WALK_TO_FIDUCIAL,
                                                                                                                                   this.walkToFiducialBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            latestFiducialPose.setIncludingFrame(locateFiducialBehavior.getFiducialPose());
            walkToFiducialBehavior.setGoalPose(latestFiducialPose);

            TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("Walking to fiducial");
            sendPacket(textToSpeechPacket);
         }
      };

      // for now just alternate between locating and walking:
      statemachine.addStateWithDoneTransition(locateFiducialBehaviorAction, FollowFiducialBehaviorStates.WALK_TO_FIDUCIAL);
      statemachine.addStateWithDoneTransition(walkToFiducialBehaviorAction, FollowFiducialBehaviorStates.LOCATE_FIDUCIAL);

      statemachine.setCurrentState(FollowFiducialBehaviorStates.LOCATE_FIDUCIAL);
   }
}
