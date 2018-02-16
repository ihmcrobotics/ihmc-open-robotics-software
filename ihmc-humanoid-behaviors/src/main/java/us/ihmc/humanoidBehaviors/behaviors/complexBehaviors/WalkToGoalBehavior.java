package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkToGoalBehaviorPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToGoalBehavior extends AbstractBehavior
{
   enum WalkToGoalBehaviorStates
   {
      WAITING_FOR_REQUEST, PLANNING, EXECUTING_PLAN
   }

   private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> walkToGoalPacketQueue = new ConcurrentListeningQueue<>(20);
   private final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> planningOutputStatusQueue = new ConcurrentListeningQueue<>(5);

   private final StateMachine<WalkToGoalBehaviorStates> stateMachine;

   private final HumanoidReferenceFrames referenceFrames;

   private final FootstepListBehavior footstepListBehavior;

   private final YoBoolean isDone;
   private final YoBoolean havePlanToExecute;
   private final YoBoolean transitionBackToWaitingState;

   private FootstepDataListMessage planToExecute;

   public WalkToGoalBehavior(CommunicationBridge outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters, YoDouble yoTime)
   {
      super(outgoingCommunicationBridge);

      stateMachine = new StateMachine<WalkToGoalBehaviorStates>("WalkToGoalBehaviorStateMachine", "WalkToGoalBehaviorStateMachineSwitchTime",
            WalkToGoalBehaviorStates.class, yoTime, registry);

      this.referenceFrames = referenceFrames;

      footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      registry.addChild(footstepListBehavior.getYoVariableRegistry());

      isDone = new YoBoolean("isDone", registry);
      havePlanToExecute = new YoBoolean("havePlanToExecute", registry);
      transitionBackToWaitingState = new YoBoolean("transitionBackToWaitingState", registry);

      attachNetworkListeningQueue(walkToGoalPacketQueue, WalkToGoalBehaviorPacket.class);
      attachNetworkListeningQueue(planningOutputStatusQueue, FootstepPlanningToolboxOutputStatus.class);

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      WaitingForRequestState waitingForRequestState = new WaitingForRequestState();
      waitingForRequestState.addStateTransition(WalkToGoalBehaviorStates.PLANNING, walkToGoalPacketQueue::isNewPacketAvailable);
      stateMachine.addState(waitingForRequestState);

      PlanningState planningState = new PlanningState();
      planningState.addStateTransition(WalkToGoalBehaviorStates.EXECUTING_PLAN, havePlanToExecute::getBooleanValue);
      planningState.addStateTransition(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST, transitionBackToWaitingState::getBooleanValue);
      stateMachine.addState(planningState);

      ExecutingPlanState executingPlanState = new ExecutingPlanState();
      executingPlanState.addStateTransition(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST, footstepListBehavior::isDone);
      stateMachine.addState(executingPlanState);

      stateMachine.setCurrentState(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST);
   }

   @Override
   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   @Override
   public void onBehaviorEntered()
   {
      walkToGoalPacketQueue.clear();
      planningOutputStatusQueue.clear();
      havePlanToExecute.set(false);
   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {
      if(stateMachine.getCurrentStateEnum().equals(WalkToGoalBehaviorStates.EXECUTING_PLAN))
         footstepListBehavior.pause();
   }

   @Override
   public void onBehaviorResumed()
   {
      if(stateMachine.getCurrentStateEnum().equals(WalkToGoalBehaviorStates.EXECUTING_PLAN))
         footstepListBehavior.resume();
   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   class WaitingForRequestState extends State<WalkToGoalBehaviorStates>
   {
      public WaitingForRequestState()
      {
         super(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST);
      }

      @Override
      public void doAction()
      {
         // Waiting for plan request
      }

      @Override
      public void doTransitionIntoAction()
      {
         // Make sure there aren't any old plan requests hanging around
         walkToGoalPacketQueue.clear();
         isDone.set(true);
         transitionBackToWaitingState.set(false);
         havePlanToExecute.set(false);
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   class PlanningState extends State<WalkToGoalBehaviorStates>
   {
      public PlanningState()
      {
         super(WalkToGoalBehaviorStates.PLANNING);
      }

      @Override
      public void doAction()
      {
         // Wait for plan
         boolean newPacketAvailable = planningOutputStatusQueue.isNewPacketAvailable();

         if(newPacketAvailable)
         {
            FootstepPlanningToolboxOutputStatus latestPacket = planningOutputStatusQueue.getLatestPacket();
            boolean validForExecution = FootstepPlanningResult.fromByte(latestPacket.planningResult).validForExecution();
            if(validForExecution)
            {
               planToExecute = latestPacket.footstepDataList;
               havePlanToExecute.set(true);
            }
            else
            {
               transitionBackToWaitingState.set(true);
            }
         }
      }

      private final Pose3D tempFinalPose = new Pose3D();
      @Override
      public void doTransitionIntoAction()
      {
         planningOutputStatusQueue.clear();
         isDone.set(false);

         ToolboxStateMessage wakeUp = MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP);
         wakeUp.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);

         communicationBridge.sendPacket(wakeUp);

         ToolboxStateMessage reinitialize = MessageTools.createToolboxStateMessage(ToolboxState.REINITIALIZE);
         reinitialize.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);

         communicationBridge.sendPacket(reinitialize);

         WalkToGoalBehaviorPacket walkToGoalBehaviorPacket = walkToGoalPacketQueue.poll();
         referenceFrames.updateFrames();
         RobotSide goalSide = RobotSide.fromByte(walkToGoalBehaviorPacket.goalSide);
         FramePose3D initialPose = new FramePose3D(referenceFrames.getSoleFrame(goalSide));
         tempFinalPose.setToZero();
         tempFinalPose.setX(walkToGoalBehaviorPacket.xGoal);
         tempFinalPose.setY(walkToGoalBehaviorPacket.yGoal);
         tempFinalPose.setOrientationYawPitchRoll(walkToGoalBehaviorPacket.thetaGoal, 0.0, 0.0);
         FramePose3D finalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), tempFinalPose);
         FootstepPlanningRequestPacket tempPlanningRequestPacket = HumanoidMessageTools.createFootstepPlanningRequestPacket(initialPose, goalSide, finalPose);
         tempPlanningRequestPacket.setTimeout(3.0);
         tempPlanningRequestPacket.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);

         communicationBridge.sendPacket(tempPlanningRequestPacket);
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   class ExecutingPlanState extends State<WalkToGoalBehaviorStates>
   {
      public ExecutingPlanState()
      {
         super(WalkToGoalBehaviorStates.EXECUTING_PLAN);
      }

      @Override
      public void doAction()
      {
         footstepListBehavior.doControl();
      }

      @Override
      public void doTransitionIntoAction()
      {
         footstepListBehavior.initialize();
         footstepListBehavior.set(planToExecute);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         footstepListBehavior.doPostBehaviorCleanup();
         isDone.set(true);
      }
   }
}
