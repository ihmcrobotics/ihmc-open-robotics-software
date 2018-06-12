package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToLocationPlannedBehavior extends StateMachineBehavior<WalkToLocationStates>
{
   public enum WalkToLocationStates
   {
      SETUP, PLAN_PATH, PLAN_FAILED, WALK_PATH, DONE
   }

   private final HumanoidReferenceFrames referenceFrames;
   private final ResetRobotBehavior resetRobotBehavior;

   private boolean walkSucceded = true;

   private FramePose3D goalPose;

   private final FootstepListBehavior footstepListBehavior;
   private final PlanPathToLocationBehavior planPathToLocationBehavior;

   public WalkToLocationPlannedBehavior(Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                        WalkingControllerParameters walkingControllerParameters, YoDouble yoTime)
   {
      super("WalkToLocationBehavior", WalkToLocationStates.class, yoTime, ros2Node);

      this.referenceFrames = referenceFrames;

      resetRobotBehavior = new ResetRobotBehavior(ros2Node, yoTime);
      footstepListBehavior = new FootstepListBehavior(ros2Node, walkingControllerParameters);
      planPathToLocationBehavior = new PlanPathToLocationBehavior(ros2Node, yoTime);
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
      BehaviorAction setUpState = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node));

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
            publishTextToSpeack("Planning Path");
         }
      };

      BehaviorAction sendPlanToController = new BehaviorAction(footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            footstepListBehavior.set(planPathToLocationBehavior.getFootStepList());
            publishTextToSpeack("Walking Path");
         }
      };
      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeack("Finished Walking");
         }
      };
      BehaviorAction planFailedState = new BehaviorAction(new SimpleDoNothingBehavior(ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            walkSucceded = false;
            publishTextToSpeack("Plan Failed");
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
