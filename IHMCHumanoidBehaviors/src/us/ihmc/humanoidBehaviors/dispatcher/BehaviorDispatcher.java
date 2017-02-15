package us.ihmc.humanoidBehaviors.dispatcher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeResponsePacket;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.communication.subscribers.RobotDataReceiver;
import us.ihmc.tools.thread.ThreadTools;

/**
 * The BehaviorDispatcher is used to select the behavior to run and to execute operator's commands as pause, resume, stop, etc.
 * DO NOT add smart AI stuff in there, create and register a new behavior in {@link IHMCHumanoidBehaviorManager} instead.
 *
 */
public class BehaviorDispatcher<E extends Enum<E>> implements Runnable
{
   private static final boolean DEBUG = true;
   private final Class<E> behaviorEnum;
   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("BehaviorDispatcher"));

   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final DoubleYoVariable yoTime;
   private final YoVariableServer yoVaribleServer;
   private final BehaviorStateMachine<E> stateMachine;

   private final EnumYoVariable<E> requestedBehavior;

   private final BehaviorTypeSubscriber<E> desiredBehaviorSubscriber;
   private final BehaviorControlModeSubscriber desiredBehaviorControlSubscriber;
   private final CommunicationBridge communicationBridge;

   private final RobotDataReceiver robotDataReceiver;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ArrayList<Updatable> updatables = new ArrayList<>();

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

   public BehaviorDispatcher(DoubleYoVariable yoTime, RobotDataReceiver robotDataReceiver, BehaviorControlModeSubscriber desiredBehaviorControlSubscriber,
         BehaviorTypeSubscriber<E> desiredBehaviorSubscriber, CommunicationBridge communicationBridge, YoVariableServer yoVaribleServer, Class<E> behaviourEnum,
         E stopBehavior, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.behaviorEnum = behaviourEnum;
      this.yoTime = yoTime;
      this.yoVaribleServer = yoVaribleServer;
      this.communicationBridge = communicationBridge;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.requestedBehavior = new EnumYoVariable<E>("requestedBehavior", registry, behaviourEnum, true);

      this.robotDataReceiver = robotDataReceiver;
      this.desiredBehaviorSubscriber = desiredBehaviorSubscriber;
      this.desiredBehaviorControlSubscriber = desiredBehaviorControlSubscriber;

      stateMachine = new BehaviorStateMachine<E>("behaviorState", "behaviorSwitchTime", behaviourEnum, yoTime, registry);

      SimpleDoNothingBehavior simpleForwardingBehavior = new SimpleDoNothingBehavior(communicationBridge);
      addBehavior(stopBehavior, simpleForwardingBehavior);
      stateMachine.setCurrentState(stopBehavior);

      requestedBehavior.set(null);

      parentRegistry.addChild(registry);
   }

   public void requestBehavior(E behaviorEnum)
   {
      requestedBehavior.set(behaviorEnum);
   }

   public void addBehaviors(List<E> Es, List<AbstractBehavior> newBehaviors)
   {
      if (Es.size() != newBehaviors.size())
         throw new RuntimeException("Arguments don't have the same size.");

      for (int i = 0; i < Es.size(); i++)
      {
         addBehavior(Es.get(i), newBehaviors.get(i));
      }
   }

   public void addBehavior(E E, AbstractBehavior behaviorToAdd)
   {
      BehaviorAction<E> behaviorStateToAdd = new BehaviorAction<E>(E, behaviorToAdd);

      this.stateMachine.addState(behaviorStateToAdd);
      this.registry.addChild(behaviorToAdd.getYoVariableRegistry());

      ArrayList<BehaviorAction<E>> allOtherBehaviorStates = new ArrayList<BehaviorAction<E>>();

      for (E otherBehaviorType : behaviorEnum.getEnumConstants())
      {
         BehaviorAction<E> otherBehaviorState = stateMachine.getState(otherBehaviorType);

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

   public void addBehaviorService(BehaviorService behaviorService)
   {
      registry.addChild(behaviorService.getYoVariableRegistry());
   }

   private void initialize()
   {
      BehaviorAction<E> currentState = stateMachine.getCurrentState();
      currentState.doTransitionIntoAction();
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
            communicationBridge.sendPacket(new BehaviorControlModeResponsePacket(BehaviorControlModeEnum.STOP));
            break;
         case PAUSE:
            stateMachine.pause();
            communicationBridge.sendPacket(new BehaviorControlModeResponsePacket(BehaviorControlModeEnum.PAUSE));
            break;
         case RESUME:
            stateMachine.resume();
            communicationBridge.sendPacket(new BehaviorControlModeResponsePacket(BehaviorControlModeEnum.RESUME));
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
      catch (Exception e)
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
      private final BehaviorAction<E> fromBehaviorState;
      private final BehaviorAction<E> toBehaviorState;

      public SwitchGlobalListenersAction(BehaviorAction<E> fromBehaviorState, BehaviorAction<E> toBehaviorState)
      {
         this.fromBehaviorState = fromBehaviorState;
         this.toBehaviorState = toBehaviorState;
      }

      @Override
      public void doTransitionAction()
      {

      }
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
