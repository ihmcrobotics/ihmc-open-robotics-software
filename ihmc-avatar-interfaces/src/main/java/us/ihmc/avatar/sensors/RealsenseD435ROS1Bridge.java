package us.ihmc.avatar.sensors;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.ihmcPerception.camera.RosCameraCompressedImageReceiver;
import us.ihmc.ihmcPerception.camera.VideoPacketHandler;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;

public class RealsenseD435ROS1Bridge
{
   private VideoPacketHandler videoPacketHandler;
   private RosCameraCompressedImageReceiver cameraImageReceiver;
   
   public RealsenseD435ROS1Bridge(RosMainNode rosMainNode, ROS2Node ros2Node)
   {
//      videoPacketHandler = new VideoPacketHandler(ros2Node, ROS2Tools.D435_VIDEO);
//      cameraImageReceiver = new RosCameraCompressedImageReceiver(cameraParameters, rosMainNode, logger, cameraReceiver);
   }
}
