package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.utilities.net.GlobalObjectConsumer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

public class BehaviorCommunicationBridge implements OutgoingCommunicationBridgeInterface, IncomingCommunicationBridgeInterface
{
   private final ObjectCommunicator networkProcessorCommunicator;
   private final ObjectCommunicator controllerCommunicator;
   private final NonBlockingGlobalObjectConsumerRelay networkProcessorToControllerRelay;
   private final NonBlockingGlobalObjectConsumerRelay controllerToNetworkProcessorRelay;
   private final BehaviorPacketPassThroughManager behaviorPacketPassThroughFromNpToController;
   
   private final YoVariableRegistry registry;
   private final BooleanYoVariable packetPassthrough;

   public BehaviorCommunicationBridge(ObjectCommunicator networkProcessorCommunicator, ObjectCommunicator controllerCommunicator,
         YoVariableRegistry parentRegistry)
   {
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.controllerCommunicator = controllerCommunicator;
      this.networkProcessorToControllerRelay = new NonBlockingGlobalObjectConsumerRelay(networkProcessorCommunicator,controllerCommunicator);
      this.controllerToNetworkProcessorRelay = new NonBlockingGlobalObjectConsumerRelay(controllerCommunicator,networkProcessorCommunicator);
      this.behaviorPacketPassThroughFromNpToController = new BehaviorPacketPassThroughManager(networkProcessorCommunicator, controllerCommunicator,
            BehaviorPacketPassthroughList.PACKETS_TO_ALWAYS_PASS_FROM_NP_TO_CONTROLLER_THROUGH_BEHAVIORS);
      
      this.
      
      registry = new YoVariableRegistry("BehaviorCommunicationBridge");
      parentRegistry.addChild(registry);
      controllerToNetworkProcessorRelay.enableForwarding();
      packetPassthrough = new BooleanYoVariable("Behavior_packetPassthrough", registry);
   }

   @Override
   public void sendPacketToController(Object obj)
   {
      networkProcessorToControllerRelay.consumeObject(obj);
   }

   @Override
   public void sendPacketToNetworkProcessor(Object obj)
   {
      controllerToNetworkProcessorRelay.consumeObject(obj);
   }

   @Override
   public void attachGlobalListenerToController(GlobalObjectConsumer listener)
   {
      controllerCommunicator.attachGlobalListener(listener);
   }

   @Override
   public void attachGlobalListenerToNetworkProcessor(GlobalObjectConsumer listener)
   {
      networkProcessorCommunicator.attachGlobalListener(listener);
   }

   @Override
   public void detachGlobalListenerFromController(GlobalObjectConsumer listener)
   {
      controllerCommunicator.detachGlobalListener(listener);
   }

   @Override
   public void detachGlobalListenerFromNetworkProcessor(GlobalObjectConsumer listener)
   {
      networkProcessorCommunicator.detachGlobalListener(listener);
   }

   public boolean isPacketPassthroughActive()
   {
      return packetPassthrough.getBooleanValue();
   }

   public void setPacketPassThrough(boolean activate)
   {
      if (!packetPassthrough.getBooleanValue() && activate)
      {
         networkProcessorToControllerRelay.enableForwarding();
         behaviorPacketPassThroughFromNpToController.setPassthroughActive(false);
      }

      if (packetPassthrough.getBooleanValue() && !activate)
      {
         networkProcessorToControllerRelay.disableForwarding();
         behaviorPacketPassThroughFromNpToController.setPassthroughActive(true);
      }
      packetPassthrough.set(activate);
   }
}
