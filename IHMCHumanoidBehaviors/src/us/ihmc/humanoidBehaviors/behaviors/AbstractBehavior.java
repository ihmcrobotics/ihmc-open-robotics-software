package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.ControllerGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.NetworkProcessorGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
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
   public static enum BehaviorStatus
   {
      INITIALIZED, PAUSED, ABORTED, DONE, FINALIZED
   }
   
   private ArrayList<AbstractBehavior> childBehaviors = new ArrayList<AbstractBehavior>();

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
   protected final BooleanYoVariable isAborted;
   protected final DoubleYoVariable percentCompleted;

   public AbstractBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this(null, outgoingCommunicationBridge);
   }

   public AbstractBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this.outgoingCommunicationBridge = outgoingCommunicationBridge;
      controllerObjectConsumer = new ControllerGlobalObjectConsumer(this);
      networkProcessorObjectConsumer = new NetworkProcessorGlobalObjectConsumer(this);
      
      behaviorName = FormattingTools.addPrefixAndKeepCamelCaseForMiddleOfExpression(namePrefix, getClass().getSimpleName());
      registry = new YoVariableRegistry(behaviorName);
      
      yoBehaviorStatus = new EnumYoVariable<AbstractBehavior.BehaviorStatus>(namePrefix + "Status", registry, BehaviorStatus.class);
      hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);
      isPaused = new BooleanYoVariable("isPaused" + behaviorName, registry);
      isAborted = new BooleanYoVariable("isAborted" + behaviorName, registry);
      percentCompleted = new DoubleYoVariable("percentCompleted", registry);
   }

   public CoactiveElement getCoactiveElement()
   {
      return null;
   }

   protected void addChildBehavior(AbstractBehavior childBehavior)
   {
      childBehaviors.add(childBehavior);
   }
   
   protected void addChildBehaviors(ArrayList<AbstractBehavior> newChildBehaviors)
   {
      for(AbstractBehavior behavior: newChildBehaviors)
      {
         childBehaviors.add(behavior);
      }
   }
   public void sendPacketToController(Packet<?> obj)
   {
      outgoingCommunicationBridge.sendPacketToController(obj);
   }

   public void sendPacketToNetworkProcessor(Packet<?> obj)
   {
      outgoingCommunicationBridge.sendPacketToNetworkProcessor(obj);
   }
   public void sendPacketToUI(Packet<?> obj)
   {
      outgoingCommunicationBridge.sendPacketToUI(obj);
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

   public void attachNetworkProcessorListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningNetworkProcessorQueues.containsKey(key))
      {
         listeningNetworkProcessorQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningNetworkProcessorQueues.get(key).add(queue);
   }
   
   public void attachControllerListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningControllerQueues.containsKey(key))
      {
         listeningControllerQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningControllerQueues.get(key).add(queue);
   }

   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (AbstractBehavior behavior : childBehaviors)
      {
         behavior.consumeObjectFromNetworkProcessor(object);
      }

   }
   
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {

      for (AbstractBehavior behavior : childBehaviors)
      {
         behavior.consumeObjectFromController(object);
      }


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

   /**
    * The implementation of this method should result in a clean shut down of the behavior.
    */
   public void abort()
   {
      isAborted.set(true);
      isPaused.set(false);
   }


   /**
    * The implementation of this method should result in pausing the behavior (pause current action and no more actions sent to the controller, the robot remains still).
    * The behavior should be resumable.
    */
   public void pause()
   {
      isPaused.set(true);
   }

   /**
    * The implementation of this method should result in resuming the behavior after being paused.
    * Should not do anything if the behavior has not been paused.
    */
   public void resume()
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

   /**
    * Clean up method that is called when leaving the behavior for another one.
    */
   public void doPostBehaviorCleanup()
   {
         isPaused.set(false);
         isAborted.set(false);
   }
   
   protected boolean isPaused()
   {
      return isPaused.getBooleanValue() || isAborted.getBooleanValue();
   }

   /**
    * Initialization method called when switching to this behavior.
    */
   @Override
   public void initialize()
   {
      isPaused.set(false);
      isAborted.set(false);
   }
   
   
   public GlobalPacketConsumer getNetworkProcessorGlobalObjectConsumer()
   {
      return networkProcessorObjectConsumer;
   }
   
   public GlobalPacketConsumer getControllerGlobalPacketConsumer()
   {
      return controllerObjectConsumer;
   }
}
