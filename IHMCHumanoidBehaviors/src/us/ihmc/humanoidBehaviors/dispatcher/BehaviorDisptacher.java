package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.List;

import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModeResponsePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleForwardingBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateWrapper;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.stateMachines.StateMachineTools;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;

/**
 * The BehaviorDispatcher is used to select the behavior to run and to execute operator's commands as pause, resume, stop, etc.
 * DO NOT add smart AI stuff in there, create and register a new behavior in {@link IHMCHumanoidBehaviorManager} instead.
 *
 */
public class BehaviorDisptacher implements Runnable
{
   private static final boolean DEBUG = true;

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable yoTime;
   private final YoVariableServer yoVaribleServer;
   private final DoubleYoVariable startTime = new DoubleYoVariable("startTime", registry);
   private final BehaviorStateMachine<HumanoidBehaviorType> stateMachine;

   private final EnumYoVariable<HumanoidBehaviorType> requestedBehavior = new EnumYoVariable<>("requestedBehavior", registry, HumanoidBehaviorType.class, true);

   private final HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber;
   private final HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber;
   private final BehaviorCommunicationBridge communicationBridge;

   private final RobotDataReceiver robotDataReceiver;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

   public BehaviorDisptacher(DoubleYoVariable yoTime, RobotDataReceiver robotDataReceiver,
         HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber, HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber,
         BehaviorCommunicationBridge communicationBridge, YoVariableServer yoVaribleServer, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoTime = yoTime;
      this.yoVaribleServer = yoVaribleServer;
      this.communicationBridge = communicationBridge;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.robotDataReceiver = robotDataReceiver;
      this.desiredBehaviorSubscriber = desiredBehaviorSubscriber;
      this.desiredBehaviorControlSubscriber = desiredBehaviorControlSubscriber;

      stateMachine = new BehaviorStateMachine<HumanoidBehaviorType>("behaviorState", "behaviorSwitchTime", HumanoidBehaviorType.class, yoTime, registry);

      SimpleForwardingBehavior simpleForwardingBehavior = new SimpleForwardingBehavior(communicationBridge, communicationBridge);
      attachListeners(simpleForwardingBehavior);
      addHumanoidBehavior(HumanoidBehaviorType.STOP, simpleForwardingBehavior);
      stateMachine.setCurrentState(HumanoidBehaviorType.STOP);

      requestedBehavior.set(null);
      startTime.set(Double.NaN);

      parentRegistry.addChild(registry);
   }

   public void addHumanoidBehaviors(List<HumanoidBehaviorType> humanoidBehaviorTypes, List<BehaviorInterface> newBehaviors)
   {
      if (humanoidBehaviorTypes.size() != newBehaviors.size())
         throw new RuntimeException("Arguments don't have the same size.");

      for (int i = 0; i < humanoidBehaviorTypes.size(); i++)
      {
         addHumanoidBehavior(humanoidBehaviorTypes.get(i), newBehaviors.get(i));
      }
   }

   public void addHumanoidBehavior(HumanoidBehaviorType humanoidBehaviorType, BehaviorInterface newBehavior)
   {
      BehaviorStateWrapper<HumanoidBehaviorType> newBehaviorState = new BehaviorStateWrapper<HumanoidBehaviorType>(humanoidBehaviorType, newBehavior);

      this.stateMachine.addState(newBehaviorState);
      this.registry.addChild(newBehavior.getYoVariableRegistry());
      // Enable transition between every existing state of the state machine
      for (HumanoidBehaviorType stateEnum : HumanoidBehaviorType.values)
      {
         BehaviorStateWrapper<HumanoidBehaviorType> otherBehavior = stateMachine.getState(stateEnum);
         if (otherBehavior == null)
            continue;

         boolean waitUntilDone = false;
         StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, new SetupBehaviorCommunication(otherBehavior, newBehaviorState), otherBehavior, newBehaviorState);
         StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, new SetupBehaviorCommunication(newBehaviorState, otherBehavior), newBehaviorState, otherBehavior);
      }
   }

   private void initialize()
   {
      stateMachine.initialize();
   }

   private void doControl()
   {
      updateRobotState();
      updateControlStatus();
      updateRequestedBehavior();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      yoGraphicsListRegistry.update();
   }

   private void updateRobotState()
   {
      robotDataReceiver.updateRobotModel();
      long simTimestamp = robotDataReceiver.getSimTimestamp();

      if (simTimestamp < 0)
         return;

      double currentTimeInSeconds = TimeTools.nanoSecondstoSeconds(simTimestamp);
      if (startTime.isNaN())
      {
         startTime.set(currentTimeInSeconds);
      }

      yoTime.set(currentTimeInSeconds - startTime.getDoubleValue());
   }

   private void updateRequestedBehavior()
   {
      if (desiredBehaviorSubscriber.checkForNewBehaviorRequested())
      {
         requestedBehavior.set(desiredBehaviorSubscriber.getRequestedBehavior());
      }
   }

   private void updateControlStatus()
   {
      if (desiredBehaviorControlSubscriber.checkForNewControlRequested())
      {
         switch (desiredBehaviorControlSubscriber.getRequestedBehaviorControl())
         {
            case STOP:
               stateMachine.stop();
               communicationBridge.sendPacketToNetworkProcessor(new HumanoidBehaviorControlModeResponsePacket(HumanoidBehaviorControlModeEnum.STOP));
               break;
            case PAUSE:
               stateMachine.pause();
               communicationBridge.setPacketPassThrough(true);
               communicationBridge.sendPacketToNetworkProcessor(new HumanoidBehaviorControlModeResponsePacket(HumanoidBehaviorControlModeEnum.PAUSE));
               break;
            case RESUME:
               stateMachine.resume();
               communicationBridge.setPacketPassThrough(false);
               communicationBridge.sendPacketToNetworkProcessor(new HumanoidBehaviorControlModeResponsePacket(HumanoidBehaviorControlModeEnum.RESUME));
               break;
            case ENABLE_ACTIONS:
               stateMachine.enableActions();
               communicationBridge.sendPacketToNetworkProcessor(new HumanoidBehaviorControlModeResponsePacket(HumanoidBehaviorControlModeEnum.ENABLE_ACTIONS));
               break;
            default:
               throw new IllegalArgumentException("BehaviorCommunicationBridge, unhandled control!");
         }
         
      }
   }

   @Override
   public void run()
   {
      while (true)
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            initialize();
            hasBeenInitialized.set(true);
         }

         doControl();

         yoVaribleServer.update(TimeTools.secondsToNanoSeconds(yoTime.getDoubleValue()));

         ThreadTools.sleep(1);
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private class SetupBehaviorCommunication implements StateTransitionAction
   {
      private final BehaviorStateWrapper<HumanoidBehaviorType> fromBehaviorState;
      private final BehaviorStateWrapper<HumanoidBehaviorType> toBehaviorState;

      public SetupBehaviorCommunication(BehaviorStateWrapper<HumanoidBehaviorType> fromBehaviorState, BehaviorStateWrapper<HumanoidBehaviorType> toBehaviorState)
      {
         this.fromBehaviorState = fromBehaviorState;
         this.toBehaviorState = toBehaviorState;
      }

      @Override
      public void doTransitionAction()
      {
         detachListeners(fromBehaviorState.getBehavior());
         attachListeners(toBehaviorState.getBehavior());
      }
   }

   private void attachListeners(BehaviorInterface behavior)
   {
      if (DEBUG)
         System.out.println("attach listeners to: " + behavior.getName());
      communicationBridge.attachGlobalListenerToController(behavior.getControllerGlobalObjectConsumer());
      communicationBridge.attachGlobalListenerToNetworkProcessor(behavior.getNetworkProcessorGlobalObjectConsumer());
   }

   private void detachListeners(BehaviorInterface behavior)
   {
      if (DEBUG)
         System.out.println("detach listeners to: " + behavior.getName());
      communicationBridge.detachGlobalListenerFromController(behavior.getControllerGlobalObjectConsumer());
      communicationBridge.detachGlobalListenerFromNetworkProcessor(behavior.getNetworkProcessorGlobalObjectConsumer());
   }
}
