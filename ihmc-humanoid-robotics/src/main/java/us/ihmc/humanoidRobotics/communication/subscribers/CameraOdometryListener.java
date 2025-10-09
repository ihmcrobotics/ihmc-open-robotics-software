package us.ihmc.humanoidRobotics.communication.subscribers;

import ihmc_common_msgs.msg.dds.StampedOdometryPacket;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.ConcurrentLinkedQueue;

public class CameraOdometryListener implements NewMessageListener<StampedOdometryPacket>
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoQuaternion imuQuaternion = new YoQuaternion("cameraOdometryIMUQuat", registry);
   private final YoPose3D cameraPose = new YoPose3D("cameraPose", registry);
   private final YoDouble cameraTimestamp = new YoDouble("cameraTimestamp", registry);
   private final ConcurrentLinkedQueue<StampedOdometryPacket> packetQueue = new ConcurrentLinkedQueue<StampedOdometryPacket>();

   public CameraOdometryListener(YoRegistry rootRegistry)
   {
      rootRegistry.addChild(registry);
   }

   @Override
   public void onNewDataMessage(Subscriber<StampedOdometryPacket> subscriber)
   {
      LogTools.info("CameraOdometryListener: New odometry message received.");
      StampedOdometryPacket newOdometryData = subscriber.takeNextData();
      imuQuaternion.set(newOdometryData.getImuOrientation());
      cameraPose.set(newOdometryData.getPose());
      cameraTimestamp.set(newOdometryData.getTimestamp());
      packetQueue.add(subscriber.takeNextData());
   }

   public StampedOdometryPacket pullPacket()
   {
      return packetQueue.poll();
   }

   public boolean isNewPacketAvailable()
   {
      return !packetQueue.isEmpty();
   }
}
