package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SearchForValveBehavior extends AbstractBehavior
{
   private RigidBodyTransform valveTransformToWorld;
   private double valveRadius;
   private boolean recievedNewValveLocation = false;

   protected final ConcurrentListeningQueue<ValveLocationPacket> valveLocationQueue = new ConcurrentListeningQueue<ValveLocationPacket>();

   public SearchForValveBehavior(CommunicationBridge behaviorCommunicationBridge)
   {
      super("SearchForSpehereFar", behaviorCommunicationBridge);
      communicationBridge.attachNetworkListeningQueue(valveLocationQueue, ValveLocationPacket.class);
   }

   @Override
   public void doControl()
   {
            if (valveLocationQueue.isNewPacketAvailable())
            {
               recievedValveLocation(valveLocationQueue.getLatestPacket());
            }
   }

   @Override
   public boolean isDone()
   {
      return recievedNewValveLocation;
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      recievedNewValveLocation = false;
   }

   public RigidBodyTransform getLocation()
   {
      return valveTransformToWorld;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }

   private void recievedValveLocation(ValveLocationPacket valveLocationPacket)
   {

      valveTransformToWorld = valveLocationPacket.getValveTransformToWorld();
      
      valveRadius = valveLocationPacket.getValveRadius();
      recievedNewValveLocation = true;

   }

}
