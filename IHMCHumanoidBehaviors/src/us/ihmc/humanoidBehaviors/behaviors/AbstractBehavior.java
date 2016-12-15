package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.tools.FormattingTools;

/**
 * Any behavior needs to implement this abstract class.
 * It helps in setting up the communications to receive and send packets to the other modules as the controller or the network processor.
 *
 */
public abstract class AbstractBehavior implements RobotController
{
   private final boolean DEBUG = false;

   public static enum BehaviorStatus
   {
      INITIALIZED, PAUSED, ABORTED, DONE, FINALIZED
   }

   protected final CommunicationBridge communicationBridge;

   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>> localListeningNetworkQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue<?>>>();

   protected final String behaviorName;

   /**
    * Every variable that can be a {@link YoVariable} should be a {@link YoVariable}, so they can be visualized in SCS.
    */
   protected final YoVariableRegistry registry;

   protected final EnumYoVariable<BehaviorStatus> yoBehaviorStatus;
   protected final BooleanYoVariable hasBeenInitialized;
   protected final BooleanYoVariable isPaused;
   protected final BooleanYoVariable isAborted;
   protected final DoubleYoVariable percentCompleted;

   private final List<BehaviorService> behaviorsServices;

   public AbstractBehavior(CommunicationBridgeInterface communicationBridge)
   {
      this(null, communicationBridge);
   }

   public AbstractBehavior(String namePrefix, CommunicationBridgeInterface communicationBridge)
   {
      this.communicationBridge = (CommunicationBridge) communicationBridge;

      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName());
      registry = new YoVariableRegistry(behaviorName);

      yoBehaviorStatus = new EnumYoVariable<AbstractBehavior.BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
      isPaused = new BooleanYoVariable("isPaused" + behaviorName, registry);
      isAborted = new BooleanYoVariable("isAborted" + behaviorName, registry);
      percentCompleted = new DoubleYoVariable("percentCompleted", registry);

      behaviorsServices = new ArrayList<>();
   }

   public void sendPacketToController(Packet<?> obj)
   {
      communicationBridge.sendPacketToController(obj);
   }

   public void sendPacket(Packet<?> obj)
   {
      communicationBridge.sendPacket(obj);
   }

   public void sendPacketToUI(Packet<?> obj)
   {
      communicationBridge.sendPacketToUI(obj);
   }

   public void attachNetworkListeningQueue(ConcurrentListeningQueue<?> queue, Class<?> key)
   {
      if (!localListeningNetworkQueues.containsKey(key))
      {
         localListeningNetworkQueues.put(key, new ArrayList<ConcurrentListeningQueue<?>>());
      }
      localListeningNetworkQueues.get(key).add(queue);
   }

   public void addBehaviorService(BehaviorService behaviorService)
   {
      behaviorsServices.add(behaviorService);
   }

   /**
    * Initialization method called when switching to this behavior.
    */
   @Override
   public final void initialize()
   {
      isPaused.set(false);
      isAborted.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.run();
      }

      addAllLocalListenersToCommunicationBridge();

      onBehaviorEntered();
   }

   public abstract void onBehaviorEntered();

   /**
    * The implementation of this method should result in a clean shut down of the behavior.
    */
   public final void abort()
   {
      isAborted.set(true);
      isPaused.set(false);
      TextToSpeechPacket p1 = new TextToSpeechPacket("Aborting Behavior");
      sendPacket(p1);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      onBehaviorAborted();
   }

   public abstract void onBehaviorAborted();

   /**
    * The implementation of this method should result in pausing the behavior (pause current action and no more actions sent to the controller, the robot remains still).
    * The behavior should be resumable.
    */
   public final void pause()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Pausing Behavior");
      sendPacket(p1);
      isPaused.set(true);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      onBehaviorPaused();
   }

   public abstract void onBehaviorPaused();

   /**
    * The implementation of this method should result in resuming the behavior after being paused.
    * Should not do anything if the behavior has not been paused.
    */
   public final void resume()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Resuming Behavior");
      sendPacket(p1);
      isPaused.set(false);
      isPaused.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.run();
      }

      onBehaviorResumed();
   }

   public abstract void onBehaviorResumed();

   /**
    * Clean up method that is called when leaving the behavior for another one.
    */
   public final void doPostBehaviorCleanup()
   {
      isPaused.set(false);
      isAborted.set(false);

      for (BehaviorService behaviorService : behaviorsServices)
      {
         behaviorService.pause();
      }

      removeAllLocalListenersFromCommunicationBridge();

      onBehaviorExited();
   }

   public abstract void onBehaviorExited();

   /**
    * Only method to check if the behavior is done.
    * @return
    */
   public abstract boolean isDone();

   protected boolean isPaused()
   {
      return isPaused.getBooleanValue() || isAborted.getBooleanValue();
   }

   private void addAllLocalListenersToCommunicationBridge()
   {
      if (DEBUG)
      {
         System.out.println("***************************************************************************");
         System.out.println("AbstractBehavior " + behaviorName + " addAllLocalListenersToCommunicationBridge");
      }
      for (Class<?> key : localListeningNetworkQueues.keySet())
      {
         for (ConcurrentListeningQueue<?> queue : localListeningNetworkQueues.get(key))
         {
            if (DEBUG)
               System.out.println("-- adding listener for " + key);
            communicationBridge.attachNetworkListeningQueue(queue, key);
         }
      }
   }

   private void removeAllLocalListenersFromCommunicationBridge()
   {
      if (DEBUG)
      {
         System.out.println("--------------------------------------------------------------------------------");
         System.out.println("AbstractBehavior " + behaviorName + " removeAllLocalListenersFromCommunicationBridge");
      }
      for (Class<?> key : localListeningNetworkQueues.keySet())
      {
         for (ConcurrentListeningQueue<?> queue : localListeningNetworkQueues.get(key))
         {
            communicationBridge.detachNetworkListeningQueue(queue, key);
         }
      }
   }

   public BehaviorStatus getBehaviorStatus()
   {
      return yoBehaviorStatus.getEnumValue();
   }

   public CommunicationBridge getCommunicationBridge()
   {
      return communicationBridge;
   }

   public CoactiveElement getCoactiveElement()
   {
      return null;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return behaviorName;
   }

   @Override
   public String getDescription()
   {
      return this.getClass().getCanonicalName();
   }
}
