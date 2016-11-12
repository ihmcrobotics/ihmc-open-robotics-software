package us.ihmc.humanoidBehaviors.behaviors.qrCode;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.qrCode.FollowQRCodes.FollowQPCodesStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;

public class FollowQRCodes extends StateMachineBehavior<FollowQPCodesStates>
{
   public enum FollowQPCodesStates
   {
      LOCATE_CODE, WALK_TOWARD_CODE, WALK_TO_CODE, SELCECT_NEXT_CODE
   }

   private final FramePose latestQPCodePose = new FramePose();

   private final LocateQRCodeBehavior locateQRCode;
   private final WalkTowardGoalBehavior walkTowardCode;

   public FollowQRCodes(DoubleYoVariable yoTime, CommunicationBridge communicationBridge)
   {
      super("FollowQPCodes", FollowQPCodesStates.class, yoTime, communicationBridge);

      locateQRCode = new LocateQRCodeBehavior(communicationBridge);
      walkTowardCode = new WalkTowardGoalBehavior(communicationBridge);
      setupStateMachine();
   }

   private void setupStateMachine()
   {
      BehaviorAction<FollowQPCodesStates> locateQRCodeAction = new BehaviorAction<FollowQPCodesStates>(FollowQPCodesStates.LOCATE_CODE, this.locateQRCode)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Looking for a QR code.");
            sendPacket(p1);
         }
      };

      BehaviorAction<FollowQPCodesStates> walkTowardCodeAction = new BehaviorAction<FollowQPCodesStates>(FollowQPCodesStates.WALK_TOWARD_CODE, this.walkTowardCode)
      {
         @Override
         protected void setBehaviorInput()
         {
            latestQPCodePose.setIncludingFrame(locateQRCode.getCodePose());
            walkTowardCode.setGoalPose(latestQPCodePose);

            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking towards the code for a couple of steps.");
            sendPacket(p1);
         }
      };

      // for now just alternate between locating and walking:
      statemachine.addStateWithDoneTransition(locateQRCodeAction, FollowQPCodesStates.WALK_TOWARD_CODE);
      statemachine.addStateWithDoneTransition(walkTowardCodeAction, FollowQPCodesStates.LOCATE_CODE);

      statemachine.setCurrentState(FollowQPCodesStates.LOCATE_CODE);
   }

}
