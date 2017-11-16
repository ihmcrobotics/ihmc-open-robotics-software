package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RepeatedlyWalkFootstepListBehavior extends AbstractBehavior
{
   enum RepeatedlyWalkFootstepListBehaviorState
   {
      WAITING_FOR_FOOTSTEP_LIST, EXECUTING_FOOTSTEP_LIST, REVERSING_FOOTSTEP_ORDER
   }

   private final StateMachine<RepeatedlyWalkFootstepListBehaviorState> stateMachine;

   private final FootstepListBehavior footstepListBehavior;

   private final ConcurrentListeningQueue<FootstepDataListMessage> footstepDataListMessageQueue = new ConcurrentListeningQueue<>(5);

   public RepeatedlyWalkFootstepListBehavior(CommunicationBridgeInterface communicationBridge, WalkingControllerParameters walkingControllerParameters,
         YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super(communicationBridge);

      stateMachine = new StateMachine<RepeatedlyWalkFootstepListBehaviorState>(getName() + "StateMachine", getName() + "StateMachineSwitchTime",
            RepeatedlyWalkFootstepListBehaviorState.class, yoTime, registry);

      footstepListBehavior = new FootstepListBehavior(communicationBridge, walkingControllerParameters);

      attachNetworkListeningQueue(footstepDataListMessageQueue, FootstepDataListMessage.class);

      setupStateMachine();

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      WaitingForFootstepListState waitingForFootstepListState = new WaitingForFootstepListState();
      waitingForFootstepListState.addStateTransition(RepeatedlyWalkFootstepListBehaviorState.EXECUTING_FOOTSTEP_LIST, footstepDataListMessageQueue::isNewPacketAvailable);
      stateMachine.addState(waitingForFootstepListState);

      ExecutingFootstepListState executingFootstepListState = new ExecutingFootstepListState();
      stateMachine.addState(executingFootstepListState);

      stateMachine.setCurrentState(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST);
   }

   @Override
   public void onBehaviorEntered()
   {

   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {
      if(!stateMachine.getCurrentStateEnum().equals(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST))
         footstepListBehavior.pause();
   }

   @Override
   public void onBehaviorResumed()
   {
      if(!stateMachine.getCurrentStateEnum().equals(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST))
         footstepListBehavior.resume();
   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doControl()
   {

   }

   class WaitingForFootstepListState extends State
   {
      public WaitingForFootstepListState()
      {
         super(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST);
      }

      @Override
      public void doAction()
      {

      }

      @Override
      public void doTransitionIntoAction()
      {

      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   class ExecutingFootstepListState extends State
   {
      public ExecutingFootstepListState()
      {
         super(RepeatedlyWalkFootstepListBehaviorState.EXECUTING_FOOTSTEP_LIST);
      }

      @Override
      public void doAction()
      {

      }

      @Override
      public void doTransitionIntoAction()
      {

      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   class ReversingFootstepOrderState extends State
   {
      public ReversingFootstepOrderState()
      {
         super(RepeatedlyWalkFootstepListBehaviorState.REVERSING_FOOTSTEP_ORDER);
      }

      @Override
      public void doAction()
      {

      }

      @Override
      public void doTransitionIntoAction()
      {

      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }
}
