package us.ihmc.gdx.ui.graphics.live;

import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.concurrent.atomic.AtomicReference;

public class GDXROS1FusedPointCloudVisualizer extends ImGuiGDXROS1Visualizer
{
   private AbstractRosTopicSubscriber<Image> zed2LeftEyeSubscriber;
   private AbstractRosTopicSubscriber<PointCloud2> l515Subscriber;
   private AbstractRosTopicSubscriber<PointCloud2> ousterSubscriber;
   AtomicReference<PointCloud2> latestL515PointCloud = new AtomicReference<>();
   AtomicReference<Image> latestZED2Image = new AtomicReference<>();
   AtomicReference<PointCloud2> latestOusterPointCloud = new AtomicReference<>();

   public GDXROS1FusedPointCloudVisualizer()
   {
      super("Fusion View");
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      zed2LeftEyeSubscriber = new AbstractRosTopicSubscriber<Image>(Image._TYPE)
      {
         @Override
         public void onNewMessage(Image image)
         {
            latestZED2Image.set(image);
         }
      };
      ros1Node.attachSubscriber(RosTools.ZED2_LEFT_EYE_VIDEO, zed2LeftEyeSubscriber);
      l515Subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            latestL515PointCloud.set(pointCloud2);
         }
      };
      ros1Node.attachSubscriber(RosTools.L515_POINT_CLOUD, l515Subscriber);
      ousterSubscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            latestOusterPointCloud.set(pointCloud2);
         }
      };
      ros1Node.attachSubscriber(RosTools.OUSTER_POINT_CLOUD, ousterSubscriber);
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(zed2LeftEyeSubscriber);
      ros1Node.removeSubscriber(l515Subscriber);
      ros1Node.removeSubscriber(ousterSubscriber);
   }
}
