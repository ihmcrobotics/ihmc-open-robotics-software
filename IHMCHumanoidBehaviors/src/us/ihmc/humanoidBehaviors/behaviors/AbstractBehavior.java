package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.GlobalObjectConsumer;
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

   protected final CommunicationBridgeInterface outgoingCommunicationBridge;
   protected final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningNetworkQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();
   private final GlobalObjectConsumer networkProcessorObjectConsumer;
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

   public AbstractBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this(null, outgoingCommunicationBridge);
   }

   public AbstractBehavior(String namePrefix, CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this.outgoingCommunicationBridge = outgoingCommunicationBridge;
      networkProcessorObjectConsumer = new GlobalObjectConsumer(this);
      
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
   
   public void consumeObjectFromNetwork(Object object)
   {
      notifyNetworkListeners(object);
      passReceivedObjectToChildBehaviors(object);
   }

   private void notifyNetworkListeners(Object object)
   {
      ArrayList<ConcurrentListeningQueue> queues = listeningNetworkQueues.get(object.getClass());
      if (queues != null)
      {
         for (int i = 0; i < queues.size(); i++)
         {
            queues.get(i).put(object);
         }
      }
   }

   public void attachNetworkListeningQueue(ConcurrentListeningQueue queue, Class<?> key)
   {
      if (!listeningNetworkQueues.containsKey(key))
      {
         listeningNetworkQueues.put(key, new ArrayList<ConcurrentListeningQueue>());
      }
      listeningNetworkQueues.get(key).add(queue);
   }
   
  

   protected void passReceivedObjectToChildBehaviors(Object object)
   {
      for (AbstractBehavior behavior : childBehaviors)
      {
         behavior.consumeObjectFromNetwork(object);
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
   
   
   public GlobalPacketConsumer getGlobalPacketConsumer()
   {
      return networkProcessorObjectConsumer;
   }
   
 
}
