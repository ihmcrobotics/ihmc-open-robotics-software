package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class ObjectWeightBehavior extends AbstractBehavior
{
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);
   private final BooleanYoVariable packetAvailable = new BooleanYoVariable("packetAvailable" + behaviorName, registry);
   private ObjectWeightPacket objectWeightPacket;
   
   public ObjectWeightBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
   }

   @Override
   public void doControl()
   {
      if (isPaused())
      {
         return;
      }
      
      if (packetAvailable.getBooleanValue())
      {
         sendPacketToController(objectWeightPacket);
         packetAvailable.set(false);
      }
   }
   
   public void setInput(ObjectWeightPacket packet)
   {
      objectWeightPacket = packet;
      packetAvailable.set(true);
      hasInputBeenSet.set(true);
   }



   @Override
   public boolean isDone()
   {
      return hasInputBeenSet() && !packetAvailable.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      super.initialize();
      hasInputBeenSet.set(false);
      packetAvailable.set(false);
   }

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

}
