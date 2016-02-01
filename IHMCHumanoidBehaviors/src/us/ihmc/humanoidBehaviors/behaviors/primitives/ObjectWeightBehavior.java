package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ObjectWeightPacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class ObjectWeightBehavior extends BehaviorInterface
{
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);
   private final BooleanYoVariable packetAvailable = new BooleanYoVariable("packetAvailable" + behaviorName, registry);
   private ObjectWeightPacket objectWeightPacket;
   
   public ObjectWeightBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
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
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void enableActions()
   {
      
   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      return hasInputBeenSet() && !packetAvailable.getBooleanValue();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      defaultInitialize();
      hasInputBeenSet.set(false);
      packetAvailable.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

}
