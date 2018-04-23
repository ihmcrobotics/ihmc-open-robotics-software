package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private FramePose3D goalPose;

   private final FootstepListBehavior footstepListBehavior;
   private final PlanPathToLocationBehavior planPathToLocationBehavior;

   public WalkToLocationPlannedBehavior(CommunicationBridge outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
                                        HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, YoDouble yoTime)
   {
      super("WalkToLocationBehavior", WalkToLocationStates.class, yoTime, outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      planPathToLocationBehavior = new PlanPathToLocationBehavior(outgoingCommunicationBridge, yoTime);
      setupStateMachine();
   }

   public void setTarget(FramePose3D targetPoseInWorld)
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

   @Override
   protected WalkToLocationStates configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkToLocationStates, BehaviorAction> factory)
   {
      BehaviorAction setUpState = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge));

      BehaviorAction planFootSteps = new BehaviorAction(planPathToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            RobotSide initialStanceSide = RobotSide.LEFT;
            RigidBodyTransform soleToWorld = referenceFrames.getSoleFrame(initialStanceSide).getTransformToWorldFrame();
            FramePose3D stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), soleToWorld);
            if (goalPose == null)
               System.err.println("WalkToLocationPlannedBehavior: goal pose NULL");

            planPathToLocationBehavior.setInputs(goalPose, stanceFootPose, initialStanceSide);
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Planning Path");
            sendPacket(p1);
         }
      };

      BehaviorAction sendPlanToController = new BehaviorAction(footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            footstepListBehavior.set(planPathToLocationBehavior.getFootStepList());
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walking Path");
            sendPacket(p1);
         }
      };
      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Finished Walking");
            sendPacket(p1);
         }
      };
      BehaviorAction planFailedState = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            walkSucceded = false;
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Plan Failed");
            sendPacket(p1);
         }
      };

      factory.addStateAndDoneTransition(WalkToLocationStates.SETUP, setUpState, WalkToLocationStates.PLAN_PATH);
      factory.addState(WalkToLocationStates.PLAN_PATH, planFootSteps);
      factory.addStateAndDoneTransition(WalkToLocationStates.WALK_PATH, sendPlanToController, WalkToLocationStates.DONE);
      factory.addStateAndDoneTransition(WalkToLocationStates.PLAN_FAILED, planFailedState, WalkToLocationStates.DONE);
      factory.addState(WalkToLocationStates.DONE, doneState);

      factory.addTransition(WalkToLocationStates.PLAN_PATH, WalkToLocationStates.WALK_PATH, t -> isPlanPathComplete() && hasValidPlanPath());
      factory.addTransition(WalkToLocationStates.PLAN_PATH, WalkToLocationStates.PLAN_FAILED, t -> isPlanPathComplete() && !hasValidPlanPath());

      return WalkToLocationStates.SETUP;
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

   private boolean isPlanPathComplete()
   {
      return planPathToLocationBehavior.isDone();
   }

   private boolean hasValidPlanPath()
   {
      return planPathToLocationBehavior.planSuccess();
   }
}
