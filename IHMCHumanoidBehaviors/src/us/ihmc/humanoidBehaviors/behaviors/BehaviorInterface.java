package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.ControllerGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.NetworkProcessorGlobalObjectConsumer;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.net.GlobalObjectConsumer;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

import us.ihmc.simulationconstructionset.robotController.RobotController;

/**
 * Any behavior needs to implement this abstract class.
 * It helps in setting up the communications to receive and send packets to the other modules as the controller or the network processor.
 *
 */
public abstract class BehaviorInterface implements RobotController
{
   protected final OutgoingCommunicationBridgeInterface outgoingCommunicationBridge;
   private final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningControllerQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();
   private final HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>> listeningNetworkProcessorQueues = new HashMap<Class<?>, ArrayList<ConcurrentListeningQueue>>();
   
   private final ControllerGlobalObjectConsumer controllerObjectConsumer;
   private final NetworkProcessorGlobalObjectConsumer networkProcessorObjectConsumer;
   protected final String behaviorName = getClass().getSimpleName();
   
   /**
    * Every variable that can be a {@link YoVariable} should be a {@link YoVariable}, so they can be visualized in SCS.
    */
   protected final YoVariableRegistry registry = new YoVariableRegistry(behaviorName);
   protected final BooleanYoVariable isPaused = new BooleanYoVariable("isPaused" + behaviorName, registry);
   protected final BooleanYoVariable isStopped = new BooleanYoVariable("isStopped" + behaviorName, registry);

   public BehaviorInterface(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      this.outgoingCommunicationBridge = outgoingCommunicationBridge;
      controllerObjectConsumer = new ControllerGlobalObjectConsumer(this);
      networkProcessorObjectConsumer = new NetworkProcessorGlobalObjectConsumer(this);
   }

   public void sendPacketToController(Object obj)
   {
      outgoingCommunicationBridge.sendPacketToController(obj);
   }

   public void sendPacketToNetworkProcessor(Object obj)
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
      return this.getClass().getSimpleName();
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

   /**
    * Needs to be clarified.
    */
   public abstract void enableActions();

   /**
    * The implementation of this method should result in pausing the behavior (pause current action and no more actions sent to the controller, the robot remains still).
    * The behavior should be resumable.
    */
   public abstract void pause();

   /**
    * The implementation of this method should result in resuming the behavior after being paused.
    * Should not do anything if the behavior has not been paused.
    */
   public abstract void resume();

   /**
    * Only method to check if the behavior is done.
    * @return
    */
   public abstract boolean isDone();

   /**
    * Clean up method that is called when leaving the behavior for another one.
    */
   public abstract void finalize();

   /**
    * Initialization method called when switching to this behavior.
    */
   public abstract void initialize();
   
   public GlobalObjectConsumer getNetworkProcessorGlobalObjectConsumer()
   {
      return networkProcessorObjectConsumer;
   }
   
   public GlobalObjectConsumer getControllerGlobalObjectConsumer()
   {
      return controllerObjectConsumer;
   }
   
   public abstract boolean hasInputBeenSet();
}
