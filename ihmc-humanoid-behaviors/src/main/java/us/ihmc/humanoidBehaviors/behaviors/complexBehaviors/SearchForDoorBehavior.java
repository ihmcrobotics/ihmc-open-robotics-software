package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;

public class SearchForDoorBehavior extends AbstractBehavior
{
   private Pose3D doorTransformToWorld;
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
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Searching For The Door");
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

   public Pose3D getLocation()
   {
      return doorTransformToWorld;
   }


   private void recievedDoorLocation(DoorLocationPacket valveLocationPacket)
   {
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Recieved Door Location From UI");
      sendPacket(p1);
      doorTransformToWorld = valveLocationPacket.getDoorTransformToWorld();

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
