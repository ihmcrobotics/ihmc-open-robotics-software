package us.ihmc.ihmcPerception.camera;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.Ros2Node;

public class VideoPacketHandler implements CompressedVideoHandler
{
   private static final boolean DEBUG = false;
   private final IHMCROS2Publisher<VideoPacket> publisher;

   private volatile boolean enable = true;

   public VideoPacketHandler(Ros2Node ros2Node)
   {
      publisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
   }

   private Stopwatch timer;
   {
      if (DEBUG)
         timer = new Stopwatch().start();
   }

   @Override
   public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation,
                       CameraPinholeBrown intrinsicParameters)
   {
      if (DEBUG)
      {
         LogTools.debug("Sending new VideoPacket FPS: " + 1.0 / timer.averageLap());
         timer.lap();
      }

      if (!enable)
         return;

      VideoPacket message = HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters);

      if (!enable)
         return;

      publisher.publish(message);
   }

   @Override
   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
   {
   }

   @Override
   public boolean isConnected()
   {
      return true;
   }

   public void closeAndDispose()
   {
      enable = false;
   }
}