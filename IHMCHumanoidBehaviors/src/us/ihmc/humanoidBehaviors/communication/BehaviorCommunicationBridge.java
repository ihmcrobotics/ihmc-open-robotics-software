package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;

public class BehaviorCommunicationBridge implements OutgoingCommunicationBridgeInterface, IncomingCommunicationBridgeInterface
{
   private final PacketCommunicator networkProcessorCommunicator;
   private final PacketCommunicator controllerCommunicator;
   private final NonBlockingGlobalObjectConsumerRelay networkProcessorToControllerRelay;
   private final NonBlockingGlobalObjectConsumerRelay controllerToNetworkProcessorRelay;
   private final BehaviorPacketPassThroughManager behaviorPacketPassThroughFromNpToController;
   
   private final YoVariableRegistry registry;
   private final BooleanYoVariable packetPassthrough;

   public BehaviorCommunicationBridge(PacketCommunicator networkProcessorCommunicator, PacketCommunicator controllerCommunicator,
         YoVariableRegistry parentRegistry)
   {
      this.networkProcessorCommunicator = networkProcessorCommunicator;
      this.controllerCommunicator = controllerCommunicator;
      this.networkProcessorToControllerRelay = new NonBlockingGlobalObjectConsumerRelay(networkProcessorCommunicator,controllerCommunicator);
      this.controllerToNetworkProcessorRelay = new NonBlockingGlobalObjectConsumerRelay(controllerCommunicator,networkProcessorCommunicator);
      this.behaviorPacketPassThroughFromNpToController = new BehaviorPacketPassThroughManager(networkProcessorCommunicator, controllerCommunicator,
            BehaviorPacketPassthroughList.PACKETS_TO_ALWAYS_PASS_FROM_NP_TO_CONTROLLER_THROUGH_BEHAVIORS);
      
      this.registry = new YoVariableRegistry("BehaviorCommunicationBridge");
      parentRegistry.addChild(registry);
      controllerToNetworkProcessorRelay.enableForwarding();
      packetPassthrough = new BooleanYoVariable("Behavior_packetPassthrough", registry);
   }

   @Override
   public void sendPacketToController(Packet obj)
   {
      networkProcessorToControllerRelay.receivedPacket(obj);
   }

   @Override
   public void sendPacketToNetworkProcessor(Packet obj)
   {
      controllerToNetworkProcessorRelay.receivedPacket(obj);
   }

   @Override
   public void attachGlobalListenerToController(PacketConsumer listener)
   {
      controllerCommunicator.attacthGlobalListener(listener);
   }

   @Override
   public void attachGlobalListenerToNetworkProcessor(PacketConsumer listener)
   {
      networkProcessorCommunicator.attacthGlobalListener(listener);
   }

   @Override
   public void detachGlobalListenerFromController(PacketConsumer listener)
   {
      controllerCommunicator.detachGlobalListener(listener);
   }

   @Override
   public void detachGlobalListenerFromNetworkProcessor(PacketConsumer listener)
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
   
   public void closeAndDispose()
   {
      networkProcessorToControllerRelay.closeAndDispose();
      controllerToNetworkProcessorRelay.closeAndDispose();
   }
}
