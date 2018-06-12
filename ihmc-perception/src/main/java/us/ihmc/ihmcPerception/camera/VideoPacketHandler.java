package us.ihmc.ihmcPerception.camera;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.VideoPacket;
import controller_msgs.msg.dds.VideoPacketPubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ros2.Ros2Node;

public class VideoPacketHandler implements CompressedVideoHandler
{
   private static final boolean DEBUG = false;
   private final IHMCROS2Publisher<VideoPacket> publisher;

   public VideoPacketHandler(Ros2Node ros2Node)
   {
      publisher = ROS2Tools.createPublisher(ros2Node, new VideoPacketPubSubType(), "/ihmc/video");
   }

   private Stopwatch timer;
   {
      if (DEBUG)
         timer = new Stopwatch().start();
   }
   
   @Override
   public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      if (DEBUG)
      {
         PrintTools.debug(DEBUG, this, "Sending new VideoPacket FPS: " + 1.0 / timer.averageLap());
         timer.lap();
      }
         
      publisher.publish(HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters));
   }

   @Override
   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
   {
//      packetCommunicator.attachStateListener(compressedVideoDataServer); 
   }

   @Override
   public boolean isConnected()
   {
      return true;
   }
}