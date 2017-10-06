package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DoorLocationPacket;

public class SearchForDoorBehavior extends AbstractBehavior
{
   private RigidBodyTransform doorTransformToWorld;
   private boolean recievedNewDoorLocation = false;

   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);

   public SearchForDoorBehavior(CommunicationBridge behaviorCommunicationBridge)
   {
      super("SearchForDoor", behaviorCommunicationBridge);
      attachNetworkListeningQueue(doorLocationQueue, DoorLocationPacket.class);
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Searching For The Door");
      sendPacket(p1);
   }

   @Override
   public void doControl()
   {
      if (doorLocationQueue.isNewPacketAvailable())
      {
         recievedDoorLocation(doorLocationQueue.getLatestPacket());
      }
   }

   @Override
   public boolean isDone()
   {
      return recievedNewDoorLocation;
   }

   @Override
   public void onBehaviorExited()
   {
      recievedNewDoorLocation = false;
   }

   public RigidBodyTransform getLocation()
   {
      return doorTransformToWorld;
   }


   private void recievedDoorLocation(DoorLocationPacket valveLocationPacket)
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Recieved Door Location From UI");
      sendPacket(p1);
      doorTransformToWorld = valveLocationPacket.getValveTransformToWorld();

      recievedNewDoorLocation = true;

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
