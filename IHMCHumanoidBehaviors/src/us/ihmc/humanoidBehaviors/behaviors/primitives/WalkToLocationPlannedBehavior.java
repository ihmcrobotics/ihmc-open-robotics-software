package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class WalkToLocationPlannedBehavior extends StateMachineBehavior<WalkToLocationStates>
{
   public enum WalkToLocationStates
   {
      SETUP, PLAN_PATH, PLAN_FAILED, WALK_PATH, DONE
   }

   private final FullRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final ResetRobotBehavior resetRobotBehavior;

   private boolean walkSucceded = true;

   private FramePose goalPose;

   private final FootstepListBehavior footstepListBehavior;
   private final PlanPathToLocationBehavior planPathToLocationBehavior;

   public WalkToLocationPlannedBehavior(CommunicationBridge outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, DoubleYoVariable yoTime)
   {
      super("WalkToLocationBehavior", WalkToLocationStates.class, yoTime, outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      planPathToLocationBehavior = new PlanPathToLocationBehavior(outgoingCommunicationBridge, yoTime);
      setUpStateMachine();

   }

   public void setTarget(FramePose targetPoseInWorld)
   {
      goalPose = targetPoseInWorld;
   }

   @Override
   public void onBehaviorEntered()
   {
      goalPose = null;
      walkSucceded = true;
      super.onBehaviorEntered();
   }

   private void setUpStateMachine()
   {

      BehaviorAction<WalkToLocationStates> setUpState = new BehaviorAction<WalkToLocationStates>(WalkToLocationStates.SETUP,
            new SimpleDoNothingBehavior(communicationBridge));

      BehaviorAction<WalkToLocationStates> planFootSteps = new BehaviorAction<WalkToLocationStates>(WalkToLocationStates.PLAN_PATH, planPathToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            RobotSide initialStanceSide = RobotSide.LEFT;
            RigidBodyTransform soleToWorld = referenceFrames.getSoleFrame(initialStanceSide).getTransformToWorldFrame();
            FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame(), soleToWorld);
            if (goalPose == null)
               System.err.println("WalkToLocationPlannedBehavior: goal pose NULL");

            planPathToLocationBehavior.setInputs(goalPose, stanceFootPose, initialStanceSide);
            TextToSpeechPacket p1 = new TextToSpeechPacket("Planning Path");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkToLocationStates> sendPlanToController = new BehaviorAction<WalkToLocationStates>(WalkToLocationStates.WALK_PATH, footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            footstepListBehavior.set(planPathToLocationBehavior.getFootStepList());
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking Path");
            sendPacket(p1);
         }
      };
      BehaviorAction<WalkToLocationStates> doneState = new BehaviorAction<WalkToLocationStates>(WalkToLocationStates.DONE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Finished Walking");
            sendPacket(p1);
         }
      };
      BehaviorAction<WalkToLocationStates> planFailedState = new BehaviorAction<WalkToLocationStates>(WalkToLocationStates.PLAN_FAILED,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            walkSucceded = false;
            TextToSpeechPacket p1 = new TextToSpeechPacket("Plan Failed");
            sendPacket(p1);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return planPathToLocationBehavior.isDone() && !planPathToLocationBehavior.planSuccess();
         }
      };
      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return planPathToLocationBehavior.isDone() && planPathToLocationBehavior.planSuccess();
         }
      };

      statemachine.addStateWithDoneTransition(setUpState, WalkToLocationStates.PLAN_PATH);

      statemachine.addState(planFootSteps);
      statemachine.addStateWithDoneTransition(sendPlanToController, WalkToLocationStates.DONE);
      statemachine.addStateWithDoneTransition(planFailedState, WalkToLocationStates.DONE);
      statemachine.addState(doneState);

      planFootSteps.addStateTransition(WalkToLocationStates.PLAN_FAILED, planFailedCondition);
      planFootSteps.addStateTransition(WalkToLocationStates.WALK_PATH, planSuccededCondition);

      //set the starting state

      statemachine.setStartState(WalkToLocationStates.SETUP);
   }

   public boolean walkSucceded()
   {
      return walkSucceded;
   }

   @Override
   public void onBehaviorAborted()
   {
      super.onBehaviorAborted();
      footstepListBehavior.onBehaviorAborted();
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {
      super.onBehaviorPaused();
      footstepListBehavior.onBehaviorPaused();
      isPaused.set(true);
   }

   @Override
   public void onBehaviorResumed()
   {
      super.onBehaviorResumed();
      footstepListBehavior.onBehaviorResumed();
      isPaused.set(false);

   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isAborted.set(false);
   }
}
