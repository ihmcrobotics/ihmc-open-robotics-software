package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleForwardingBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateWrapper;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorControlModeResponsePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.subscribers.RobotDataReceiver;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.StateMachineTools;
import us.ihmc.robotics.stateMachines.StateTransitionAction;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * The BehaviorDispatcher is used to select the behavior to run and to execute operator's commands as pause, resume, stop, etc.
 * DO NOT add smart AI stuff in there, create and register a new behavior in {@link IHMCHumanoidBehaviorManager} instead.
 *
 */
public class BehaviorDisptacher implements Runnable
{
   private static final boolean DEBUG = true;
   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("BehaviorDispatcher"));
   
   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable yoTime;
   private final YoVariableServer yoVaribleServer;
   private final BehaviorStateMachine<HumanoidBehaviorType> stateMachine;

   private final EnumYoVariable<HumanoidBehaviorType> requestedBehavior = new EnumYoVariable<>("requestedBehavior", registry, HumanoidBehaviorType.class, true);

   private final HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber;
   private final HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber;
   private final BehaviorCommunicationBridge communicationBridge;

   private final RobotDataReceiver robotDataReceiver;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ArrayList<Updatable> updatables = new ArrayList<>();

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

      SimpleForwardingBehavior simpleForwardingBehavior = new SimpleForwardingBehavior(communicationBridge);
      attachListeners(simpleForwardingBehavior);
      addHumanoidBehavior(HumanoidBehaviorType.STOP, simpleForwardingBehavior);
      stateMachine.setCurrentState(HumanoidBehaviorType.STOP);

      requestedBehavior.set(null);

      parentRegistry.addChild(registry);
   }

   public void requestBehavior(HumanoidBehaviorType behaviorEnum)
   {
      requestedBehavior.set(behaviorEnum);
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

   public void addHumanoidBehavior(HumanoidBehaviorType humanoidBehaviorType, BehaviorInterface behaviorToAdd)
   {
      BehaviorStateWrapper<HumanoidBehaviorType> behaviorStateToAdd = new BehaviorStateWrapper<HumanoidBehaviorType>(humanoidBehaviorType, behaviorToAdd);

      this.stateMachine.addState(behaviorStateToAdd);
      this.registry.addChild(behaviorToAdd.getYoVariableRegistry());

      ArrayList<BehaviorStateWrapper<HumanoidBehaviorType>> allOtherBehaviorStates = new ArrayList<BehaviorStateWrapper<HumanoidBehaviorType>>();

      for (HumanoidBehaviorType otherBehaviorType : HumanoidBehaviorType.values)
      {
         BehaviorStateWrapper<HumanoidBehaviorType> otherBehaviorState = stateMachine.getState(otherBehaviorType);

         if (otherBehaviorState == null)
         {
            continue;
         }
         else
         {
            allOtherBehaviorStates.add(otherBehaviorState);

            boolean waitUntilDone = false;

            SwitchGlobalListenersAction switchToOtherBehaviorState = new SwitchGlobalListenersAction(behaviorStateToAdd, otherBehaviorState);
            StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, switchToOtherBehaviorState, behaviorStateToAdd, otherBehaviorState);

            SwitchGlobalListenersAction switchFromOtherBehaviorState = new SwitchGlobalListenersAction(otherBehaviorState, behaviorStateToAdd);
            StateMachineTools.addRequestedStateTransition(requestedBehavior, waitUntilDone, switchFromOtherBehaviorState, otherBehaviorState,
                  behaviorStateToAdd);
         }
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
      callUpdatables();

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();

      yoGraphicsListRegistry.update();
   }

   private void callUpdatables()
   {
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(yoTime.getDoubleValue());
      }
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   private void updateRobotState()
   {
      robotDataReceiver.updateRobotModel();
      long simTimestamp = robotDataReceiver.getSimTimestamp();

      if (simTimestamp < 0)
         return;

      double currentTimeInSeconds = TimeTools.nanoSecondstoSeconds(simTimestamp);
      yoTime.set(currentTimeInSeconds);
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
      try
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            initialize();
            hasBeenInitialized.set(true);
         }
   
         doControl();
   
         if (yoVaribleServer != null)
         {
            yoVaribleServer.update(TimeTools.secondsToNanoSeconds(yoTime.getDoubleValue()));
         }
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private class SwitchGlobalListenersAction implements StateTransitionAction
   {
      private final BehaviorStateWrapper<HumanoidBehaviorType> fromBehaviorState;
      private final BehaviorStateWrapper<HumanoidBehaviorType> toBehaviorState;

      public SwitchGlobalListenersAction(BehaviorStateWrapper<HumanoidBehaviorType> fromBehaviorState,
            BehaviorStateWrapper<HumanoidBehaviorType> toBehaviorState)
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
         System.out.println(this.getClass().getSimpleName() + ": attach listeners to: " + behavior.getName());
      communicationBridge.attachGlobalListener(behavior.getControllerGlobalPacketConsumer());
      communicationBridge.attachGlobalListener(behavior.getNetworkProcessorGlobalObjectConsumer());
   }

   private void detachListeners(BehaviorInterface behavior)
   {
      if (DEBUG)
         System.out.println(this.getClass().getSimpleName() + ": detach listeners to: " + behavior.getName());
      communicationBridge.detachGlobalListener(behavior.getControllerGlobalPacketConsumer());
      communicationBridge.detachGlobalListener(behavior.getNetworkProcessorGlobalObjectConsumer());
   }
   
   public void start()
   {
      // do start
      scheduler.scheduleAtFixedRate(this, 0, 10, TimeUnit.MILLISECONDS);
   }
   
   public void closeAndDispose()
   {
      
      // do stop
      scheduler.shutdown();
      try
      {
         scheduler.awaitTermination(10, TimeUnit.SECONDS);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException("Cannot shutdown BehaviorDispatcher", e);
      }
   }
}
