package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;

public class SearchForValveBehavior extends AbstractBehavior
{
   private Pose3D valveTransformToWorld;
   private double valveRadius;
   private boolean recievedNewValveLocation = false;

   protected final ConcurrentListeningQueue<ValveLocationPacket> valveLocationQueue = new ConcurrentListeningQueue<ValveLocationPacket>(10);

   public SearchForValveBehavior(CommunicationBridge behaviorCommunicationBridge)
   {
      super("SearchForSpehereFar", behaviorCommunicationBridge);
      attachNetworkListeningQueue(valveLocationQueue, ValveLocationPacket.class);
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Searching For The Valve");
      sendPacket(p1);
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
   public void onBehaviorExited()
   {
      recievedNewValveLocation = false;
   }

   public Pose3D getLocation()
   {
      return valveTransformToWorld;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }

   private void recievedValveLocation(ValveLocationPacket valveLocationPacket)
   {
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Recieved Valve Location From UI");
      sendPacket(p1);
      valveTransformToWorld = valveLocationPacket.getValveTransformToWorld();

      valveRadius = valveLocationPacket.getValveRadius();
      recievedNewValveLocation = true;

   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}
