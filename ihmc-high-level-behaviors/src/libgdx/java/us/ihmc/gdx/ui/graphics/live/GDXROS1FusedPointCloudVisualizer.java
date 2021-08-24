package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

public class GDXROS1FusedPointCloudVisualizer extends ImGuiGDXROS1Visualizer
{
   private static final int MAX_POINTS = 250000;
   private AbstractRosTopicSubscriber<Image> zed2LeftEyeSubscriber;
   private AbstractRosTopicSubscriber<PointCloud2> l515Subscriber;
   private AbstractRosTopicSubscriber<PointCloud2> ousterSubscriber;
   AtomicReference<PointCloud2> latestL515PointCloud = new AtomicReference<>();
   AtomicReference<Image> latestZED2Image = new AtomicReference<>();
   AtomicReference<PointCloud2> latestOusterPointCloud = new AtomicReference<>();
   private final GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
   private final ReferenceFrame ousterFrame;
   private final ReferenceFrame l515Frame;
   private final ReferenceFrame zed2Frame;
   private ByteBuffer l515PointsOnlyBuffer;

   public GDXROS1FusedPointCloudVisualizer(HumanoidReferenceFrames referenceFrames)
   {
      super("Fusion View");
      l515Frame = referenceFrames.getSteppingCameraFrame();
      ousterFrame = referenceFrames.getObjectDetectionCameraFrame();
      zed2Frame = referenceFrames.getHeadZED2CameraFrame();
   }

   @Override
   public void create()
   {
      super.create();
      pointCloudRenderer.create(MAX_POINTS);
   }

   @Override
   public void update()
   {
      super.update();

      PointCloud2 ousterPointCloud2 = latestOusterPointCloud.get();
      PointCloud2 l515PointCloud2 = latestL515PointCloud.get();
      Image zed2Image = latestZED2Image.get();

      if (ousterPointCloud2 != null && l515PointCloud2 != null && zed2Image != null)
      {
         long ousterTimestamp = ousterPointCloud2.getHeader().getStamp().totalNsecs();


         ChannelBuffer l515NettyImageData = l515PointCloud2.getData();
         ByteBuffer l515DataByteBuffer = l515NettyImageData.toByteBuffer();
         int arrayOffset = l515NettyImageData.arrayOffset();
         l515DataByteBuffer.position(arrayOffset);
         ByteBuffer offsetByteBuffer = l515DataByteBuffer.slice();
         BytePointer imageDataPointer = new BytePointer(offsetByteBuffer);
         Mat l515WithRGB = new Mat(l515PointCloud2.getHeight(), l515PointCloud2.getWidth(), opencv_core.CV_32FC4, imageDataPointer);
         if (l515PointsOnlyBuffer == null)
            l515PointsOnlyBuffer = BufferUtils.newByteBuffer(l515PointCloud2.getWidth() * l515PointCloud2.getHeight() * 3);
         Mat l515PointsOnly = new Mat(l515PointCloud2.getHeight(), l515PointCloud2.getWidth(), opencv_core.CV_32FC3, new BytePointer(l515PointsOnlyBuffer));
         int[] fromTo = new int[] {0, 0, 1, 1, 2, 2};
         opencv_core.mixChannels(l515WithRGB, 4, l515PointsOnly, 3, fromTo, 3);

         // floats: X,Y,Z,R,G,B,A,0.01,1.0,0.0
         float[] coloredPointDataArray = pointCloudRenderer.getVerticesArray(); // Copy data to this array

         pointCloudRenderer.updateMeshFast(l515PointCloud2.getWidth() * l515PointCloud2.getHeight()
                                           + ousterPointCloud2.getWidth() * ousterPointCloud2.getHeight());
      }
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

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }
}
