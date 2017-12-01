package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import com.google.common.collect.Lists;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class RepeatedlyWalkFootstepListBehavior extends AbstractBehavior
{
   enum RepeatedlyWalkFootstepListBehaviorState
   {
      WAITING_FOR_FOOTSTEP_LIST, EXECUTING_FOOTSTEP_LIST, REVERSING_FOOTSTEP_ORDER, REPEATING_FOOTSTEP_LIST
   }

   private final StateMachine<RepeatedlyWalkFootstepListBehaviorState> stateMachine;

   private final FootstepListBehavior footstepListBehavior;

   private final ConcurrentListeningQueue<FootstepDataListMessage> footstepDataListMessageQueue = new ConcurrentListeningQueue<>(5);
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusMessageQueue = new ConcurrentListeningQueue<>(20);

   private final YoBoolean finishedCurrentFootstepList;

   private FootstepDataListMessage footstepListToRepeat;

   public RepeatedlyWalkFootstepListBehavior(CommunicationBridgeInterface communicationBridge, WalkingControllerParameters walkingControllerParameters,
         YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super(communicationBridge);

      stateMachine = new StateMachine<>(getName() + "StateMachine", getName() + "StateMachineSwitchTime",
            RepeatedlyWalkFootstepListBehaviorState.class, yoTime, registry);

      footstepListBehavior = new FootstepListBehavior(communicationBridge, walkingControllerParameters);

      finishedCurrentFootstepList = new YoBoolean("finishedCurrentFootstepList", registry);

      attachNetworkListeningQueue(footstepDataListMessageQueue, FootstepDataListMessage.class);
      attachNetworkListeningQueue(walkingStatusMessageQueue, WalkingStatusMessage.class);

      setupStateMachine();

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      WaitingForFootstepListState waitingForFootstepListState = new WaitingForFootstepListState();
      waitingForFootstepListState.addStateTransition(RepeatedlyWalkFootstepListBehaviorState.EXECUTING_FOOTSTEP_LIST, footstepDataListMessageQueue::isNewPacketAvailable);
      stateMachine.addState(waitingForFootstepListState);

      ExecutingFootstepListState executingFootstepListState = new ExecutingFootstepListState();
      executingFootstepListState.addStateTransition(RepeatedlyWalkFootstepListBehaviorState.REVERSING_FOOTSTEP_ORDER, finishedCurrentFootstepList::getBooleanValue);
      stateMachine.addState(executingFootstepListState);

      ReversingFootstepOrderState reversingFootstepOrderState = new ReversingFootstepOrderState();
      reversingFootstepOrderState.addImmediateStateTransition(RepeatedlyWalkFootstepListBehaviorState.REPEATING_FOOTSTEP_LIST);
      stateMachine.addState(reversingFootstepOrderState);

      RepeatingFootstepListState repeatingFootstepListState = new RepeatingFootstepListState();
      repeatingFootstepListState.addStateTransition(RepeatedlyWalkFootstepListBehaviorState.REVERSING_FOOTSTEP_ORDER, footstepListBehavior::isDone);
      stateMachine.addState(repeatingFootstepListState);

      stateMachine.setCurrentState(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST);
   }

   @Override
   public void onBehaviorEntered()
   {

   }

   @Override
   public void onBehaviorAborted()
   {
      stateMachine.setCurrentState(RepeatedlyWalkFootstepListBehaviorState.WAITING_FOR_FOOTSTEP_LIST);
      finishedCurrentFootstepList.set(true);
      footstepDataListMessageQueue.clear();
      walkingStatusMessageQueue.clear();
      footstepListBehavior.onBehaviorAborted();
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
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
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
         footstepDataListMessageQueue.clear();
         finishedCurrentFootstepList.set(true);
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
         if(walkingStatusMessageQueue.isNewPacketAvailable())
         {
            WalkingStatusMessage latestPacket = walkingStatusMessageQueue.getLatestPacket();
            WalkingStatusMessage.Status walkingStatus = latestPacket.getWalkingStatus();
            if(walkingStatus.equals(WalkingStatusMessage.Status.COMPLETED))
            {
               finishedCurrentFootstepList.set(true);
            }
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         finishedCurrentFootstepList.set(false);
         FootstepDataListMessage latestPacket = footstepDataListMessageQueue.getLatestPacket();
         footstepListToRepeat = latestPacket;
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
         finishedCurrentFootstepList.set(false);
         List<FootstepDataMessage> reversedFootstepList = Lists.reverse(footstepListToRepeat.footstepDataList);
         footstepListToRepeat.footstepDataList = new ArrayList<>();
         footstepListToRepeat.footstepDataList.addAll(reversedFootstepList);
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   class RepeatingFootstepListState extends State
   {
      public RepeatingFootstepListState()
      {
         super(RepeatedlyWalkFootstepListBehaviorState.REPEATING_FOOTSTEP_LIST);
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
         footstepListBehavior.set(footstepListToRepeat);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         footstepListBehavior.doPostBehaviorCleanup();
      }
   }
}
