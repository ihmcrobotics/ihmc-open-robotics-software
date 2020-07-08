package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.Ros2Node;

public class SearchForDoorBehavior extends AbstractBehavior
{
   private Pose3D doorTransformToWorld;
   private byte detectedDoorType;

   private boolean recievedNewDoorLocation = false;

   protected final ConcurrentListeningQueue<DoorLocationPacket> doorLocationQueue = new ConcurrentListeningQueue<DoorLocationPacket>(10);

   public SearchForDoorBehavior(String robotName, String yoNamePrefix, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, yoNamePrefix, ros2Node);

      createSubscriber(DoorLocationPacket.class, ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput(), doorLocationQueue::put);

   }

   @Override
   public void onBehaviorEntered()
   {
      doorLocationQueue.clear();
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
      return recievedNewDoorLocation && doorTransformToWorld != null;
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

   public byte getDoorType()
   {
      return detectedDoorType;
   }

   private void recievedDoorLocation(DoorLocationPacket doorLocationPacket)
   {
      setDoorType(doorLocationPacket.detected_door_type_);
      setDoorLocation(doorLocationPacket.getDoorTransformToWorld());
   }

   public void setDoorLocation(Pose3D pose)
   {
      System.out.println("SET DOOR LOCATION " + pose);
      doorTransformToWorld = pose;
      recievedNewDoorLocation = true;
   }

   public void setDoorType(byte doorType)
   {
      detectedDoorType = doorType;
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
