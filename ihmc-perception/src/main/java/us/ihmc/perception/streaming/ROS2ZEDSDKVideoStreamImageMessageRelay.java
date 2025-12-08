package us.ihmc.perception.streaming;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotDataLogger.ZEDSDKAnnounce;
import us.ihmc.robotDataLogger.logger.ZEDSVOLoggerManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.io.Closeable;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class ROS2ZEDSDKVideoStreamImageMessageRelay implements Closeable
{
   private static final boolean ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();

   private final Map<String, ROS2ZEDSDKVideoStreamImageMessageRelayWorker> workers = new HashMap<>();

   private final ROS2Subscription<ZEDSDKAnnounce> zedSDKAnnounceSubscription;

   public ROS2ZEDSDKVideoStreamImageMessageRelay(ROS2Node ros2Node, int slDepthMode, Function<String, ReferenceFrame> referenceFrameProvider)
   {
      zedSDKAnnounceSubscription = ros2Node.createSubscription2(ZEDSVOLoggerManager.ZED_SDK_ANNOUNCE_TOPIC, announceMessage ->
      {
         if (ZED_SDK_LOADED && !workers.containsKey(announceMessage.getSensorNameAsString()))
         {
            ROS2ZEDSDKVideoStreamImageMessageRelayWorker worker = new ROS2ZEDSDKVideoStreamImageMessageRelayWorker(ros2Node,
                                                                                                                   ZEDModelData.ZED_X_MINI,
                                                                                                                   slDepthMode,
                                                                                                                   announceMessage.getAddressAsString(),
                                                                                                                   announceMessage.getPort());
            worker.setSensorFrame(referenceFrameProvider.apply(announceMessage.getSensorNameAsString()));
            worker.run(true);

            workers.put(announceMessage.getSensorNameAsString(), worker);
         }
      });
   }

   @Override
   public void close()
   {
      for (ROS2ZEDSDKVideoStreamImageMessageRelayWorker worker : workers.values())
         worker.close();

      zedSDKAnnounceSubscription.remove();
   }
}
