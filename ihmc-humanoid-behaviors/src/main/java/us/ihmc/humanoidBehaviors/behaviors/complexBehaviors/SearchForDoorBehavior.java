package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.DoorLocationPacketPubSubType;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.Ros2Node;

public class SearchForDoorBehavior extends AbstractBehavior
{
   private Pose3D doorTransformToWorld;
   private boolean recievedNewDoorLocation = false;

   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);

   public SearchForDoorBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, "SearchForDoor", ros2Node);
      createSubscriber(doorLocationQueue, new DoorLocationPacketPubSubType(), "/ihmc/door_location");
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeack("Searching For The Door");
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
      publishTextToSpeack("Recieved Door Location From UI");
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
