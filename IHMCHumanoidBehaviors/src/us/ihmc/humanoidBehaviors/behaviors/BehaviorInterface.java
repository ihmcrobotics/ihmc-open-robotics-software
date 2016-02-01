package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.ControllerGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.NetworkProcessorGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.tools.FormattingTools;

/**
 * Any behavior needs to implement this abstract class.
 * It helps in setting up the communications to receive and send packets to the other modules as the controller or the network processor.
 *
 */
public abstract class BehaviorInterface implements RobotController
{
   public static enum BehaviorStatus
   {
      INITIALIZED, PAUSED, STOPPED, DONE, FINALIZED
   }
   
   protected final OutgoingCommunicationBridgeInterface outgoingCommunicationBridge;
   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningControllerQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();
   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningNetworkProcessorQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();

   private final ControllerGlobalObjectConsumer controllerObjectConsumer;
   private final NetworkProcessorGlobalObjectConsumer networkProcessorObjectConsumer;
   protected final String behaviorName;
   
   /**
    * Every variable that can be a {@link YoVariable} should be a {@link YoVariable}, so they can be visualized in SCS.
    */
   protected final YoVariableRegistry registry;
   
   protected final EnumYoVariable<BehaviorStatus> yoBehaviorStatus;
   protected final BooleanYoVariable hasBeenInitialized;
   protected final BooleanYoVariable isPaused;
   protected final BooleanYoVariable isStopped;
   protected final DoubleYoVariable percentCompleted;

   public BehaviorInterface(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this(null, outgoingCommunicationBridge);
   }

   public BehaviorInterface(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this.outgoingCommunicationBridge = outgoingCommunicationBridge;
      controllerObjectConsumer = new ControllerGlobalObjectConsumer(this);
      networkProcessorObjectConsumer = new NetworkProcessorGlobalObjectConsumer(this);
      
      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName());
      registry = new YoVariableRegistry(behaviorName);
      
      yoBehaviorStatus = new EnumYoVariable<BehaviorInterface.BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
      isPaused = new BooleanYoVariable("isPaused" + behaviorName, registry);
      isStopped = new BooleanYoVariable("isStopped" + behaviorName, registry);
      percentCompleted = new DoubleYoVariable("percentCompleted", registry);
   }

   public void sendPacketToController(Packet<?> obj)
   {
      outgoingCommunicationBridge.sendPacketToController(obj);
   }

   public void sendPacketToNetworkProcessor(Packet<?> obj)
   {
      outgoingCommunicationBridge.sendPacketToNetworkProcessor(obj);
   }
   
   public void consumeObjectFromController(Object object)
   {
      notifyControllerListeners(object);
      passReceivedControllerObjectToChildBehaviors(object);
   }

   public void consumeObjectFromNetworkProcessor(Object object)
   {
      notifyNetworkProcessorListeners(object);
      passReceivedNetworkProcessorObjectToChildBehaviors(object);
   }

   private void notifyControllerListeners(Object object)
   {
      ArrayList<ConcurrentListeningQueue> queues = listeningControllerQueues.get(object.getClass());
      if (queues != null)
      {
         for (int i = 0; i < queues.size(); i++)
         {
            queues.get(i).put(object);
         }
      }
   }
   
   private void notifyNetworkProcessorListeners(Object object)
   {
      ArrayList<ConcurrentListeningQueue> queues = listeningNetworkProcessorQueues.get(object.getClass());
      if (queues != null)
      {
         for (int i = 0; i < queues.size(); i++)
         {
            queues.get(i).put(object);
         }
      }
   }

   protected void attachNetworkProcessorListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningNetworkProcessorQueues.containsKey(key))
      {
         listeningNetworkProcessorQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningNetworkProcessorQueues.get(key).add(queue);
   }
   
   protected void attachControllerListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningControllerQueues.containsKey(key))
      {
         listeningControllerQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningControllerQueues.get(key).add(queue);
   }

   protected abstract void passReceivedNetworkProcessorObjectToChildBehaviors(Object object);
   
   protected abstract void passReceivedControllerObjectToChildBehaviors(Object object);

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

   /**
    * The implementation of this method should result in a clean shut down of the behavior.
    */
   public abstract void stop();
   public void defaultStop()
   {
      isStopped.set(true);
      isPaused.set(false);
   }

   /**
    * Needs to be clarified.
    */
   public abstract void enableActions();

   /**
    * The implementation of this method should result in pausing the behavior (pause current action and no more actions sent to the controller, the robot remains still).
    * The behavior should be resumable.
    */
   public abstract void pause();
   public void defaultPause()
   {
      isPaused.set(true);
   }

   /**
    * The implementation of this method should result in resuming the behavior after being paused.
    * Should not do anything if the behavior has not been paused.
    */
   public abstract void resume();
   public void defaultResume()
   {
      isPaused.set(false);
   }

   
   public BehaviorStatus getBehaviorStatus()
   {
      return yoBehaviorStatus.getEnumValue();
   }
   
   /**
    * Only method to check if the behavior is done.
    * @return
    */
   public abstract boolean isDone();
   public boolean defaultIsDone()
   {
      boolean ret = !isPaused.getBooleanValue() && !isStopped.getBooleanValue();
      return ret;
   }

   /**
    * Clean up method that is called when leaving the behavior for another one.
    */
   public abstract void doPostBehaviorCleanup();
   public void defaultPostBehaviorCleanup()
   {
      isPaused.set(false);
      isStopped.set(false);
   }
   
   protected boolean isPaused()
   {
      return isPaused.getBooleanValue() || isStopped.getBooleanValue();
   }

   /**
    * Initialization method called when switching to this behavior.
    */
   public abstract void initialize();
   public void defaultInitialize()
   {
      isPaused.set(false);
      isStopped.set(false);
   }
   
   
   public GlobalPacketConsumer getNetworkProcessorGlobalObjectConsumer()
   {
      return networkProcessorObjectConsumer;
   }
   
   public GlobalPacketConsumer getControllerGlobalPacketConsumer()
   {
      return controllerObjectConsumer;
   }
   
   public abstract boolean hasInputBeenSet();
}
