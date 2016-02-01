package us.ihmc.humanoidBehaviors.communication;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class BehaviorCommunicationBridge implements OutgoingCommunicationBridgeInterface, IncomingCommunicationBridgeInterface
{
   private final PacketCommunicator behaviorPacketCommunicator;
   
   
   
   
//   private final PacketCommunicator networkProcessorCommunicator;
//   private final PacketCommunicator controllerCommunicator;
//   private final NonBlockingGlobalObjectConsumerRelay networkProcessorToControllerRelay;
//   private final NonBlockingGlobalObjectConsumerRelay controllerToNetworkProcessorRelay;
//   private final BehaviorPacketPassThroughManager behaviorPacketPassThroughFromNpToController;
   
   private final YoVariableRegistry registry;
   private final BooleanYoVariable packetPassthrough;

//   public BehaviorCommunicationBridge(PacketCommunicator networkProcessorCommunicator, PacketCommunicator controllerCommunicator,
//         YoVariableRegistry parentRegistry)
//   {
//      this.networkProcessorCommunicator = networkProcessorCommunicator;
//      this.controllerCommunicator = controllerCommunicator;
//      this.networkProcessorToControllerRelay = new NonBlockingGlobalObjectConsumerRelay(networkProcessorCommunicator,controllerCommunicator);
//      this.controllerToNetworkProcessorRelay = new NonBlockingGlobalObjectConsumerRelay(controllerCommunicator,networkProcessorCommunicator);
//      this.behaviorPacketPassThroughFromNpToController = new BehaviorPacketPassThroughManager(networkProcessorCommunicator, controllerCommunicator,
//            BehaviorPacketPassthroughList.PACKETS_TO_ALWAYS_PASS_FROM_NP_TO_CONTROLLER_THROUGH_BEHAVIORS);
//      
//      
//      
//      this.registry = new YoVariableRegistry("BehaviorCommunicationBridge");
//      parentRegistry.addChild(registry);
//      controllerToNetworkProcessorRelay.enableForwarding();
//      packetPassthrough = new BooleanYoVariable("Behavior_packetPassthrough", registry);
//   }

   public BehaviorCommunicationBridge(PacketCommunicator behaviorPacketCommunicator, YoVariableRegistry parentRegistry)
   {
      this.behaviorPacketCommunicator = behaviorPacketCommunicator; 
      this.registry = new YoVariableRegistry("BehaviorCommunicationBridge");
      parentRegistry.addChild(registry);
      packetPassthrough = new BooleanYoVariable("Behavior_packetPassthrough", registry);
   }

   @Override
   public void sendPacketToController(Packet packet)
   {
      packet.setDestination(PacketDestination.CONTROLLER.ordinal());
      behaviorPacketCommunicator.send(packet);
   }

   @Override
   public void sendPacketToNetworkProcessor(Packet obj)
   {
      behaviorPacketCommunicator.send(obj);
   }

   @Override
   public void attachGlobalListener(GlobalPacketConsumer listener)
   {
      behaviorPacketCommunicator.attachGlobalListener(listener);
   }

   @Override
   public void detachGlobalListener(GlobalPacketConsumer listener)
   {
      behaviorPacketCommunicator.detachGlobalListener(listener);
   }

   public boolean isPacketPassthroughActive()
   {
      return packetPassthrough.getBooleanValue();
   }

   public void setPacketPassThrough(boolean activate)
   {
//      if (!packetPassthrough.getBooleanValue() && activate)
//      {
//         networkProcessorToControllerRelay.enableForwarding();
//         behaviorPacketPassThroughFromNpToController.setPassthroughActive(false);
//      }
//
//      if (packetPassthrough.getBooleanValue() && !activate)
//      {
//         networkProcessorToControllerRelay.disableForwarding();
//         behaviorPacketPassThroughFromNpToController.setPassthroughActive(true);
//      }
//      packetPassthrough.set(activate);
   }
   
   public void closeAndDispose()
   {
//      networkProcessorToControllerRelay.closeAndDispose();
//      controllerToNetworkProcessorRelay.closeAndDispose();
   }
}
