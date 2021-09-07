package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencl.*;
import org.bytedeco.opencv.opencv_core.Mat;
import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
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
   private long l515ReceivedCount = 0;
   private long zed2ReceivedCount = 0;
   private long ousterReceivedCount = 0;
   private final ImGuiPlot l515ReceivedPlot = new ImGuiPlot("", 1000, 230, 20);
   private final ImGuiPlot zed2ReceivedPlot = new ImGuiPlot("", 1000, 230, 20);
   private final ImGuiPlot ousterReceivedPlot = new ImGuiPlot("", 1000, 230, 20);
   private final ReferenceFrame ousterFrame;
   private final ReferenceFrame l515Frame;
   private final ReferenceFrame zed2Frame;
   private ByteBuffer l515PointsOnlyHostBuffer;
   private int[] l515RetainXYZChannels;
   private Mat l515WithRGB;
   private Mat l515PointsOnly;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private FloatBuffer coloredPointCloudDataHostFloatBuffer;
   private _cl_mem ousterInGPUBuffer;
   private _cl_mem fusedOutGPUBuffer;
   private _cl_kernel projectZED2ToOusterPointsKernel;
   private int ousterInBytesLength;
   private int fusedOutBytesLength;
   private _cl_mem zed2InGPUBuffer;
   private int zed2InBytesLength;
   private int numberOfOusterPoints;
   private ByteBuffer zed2InHostBuffer;
   private ByteBuffer ousterInHostBuffer;

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

      // ouster width 1024 height 128, 9 fields, 288 bits, 36 bytes, BUT! 48 bytes step
      // 3 float32s XYZ
      // 1 float 32 intensity
      // 32 bits unused
      // unint32 t(time?),
      // unint16 reflectivity
      // uint8 ring
      // 8 bits unused
      // unint16 ambient
      // 16 bits unused
      // unint32 range
      // 12 byte waste
      // l515 width 151413 height 1, 4 float XYX(RGB), uint16 ambient
      // zed2 1280x720, bgr8

      l515WithRGB = new Mat(1, 151413, opencv_core.CV_32FC4);
      l515RetainXYZChannels = new int[] {0, 0, 1, 1, 2, 2};
      l515PointsOnlyHostBuffer = BufferUtils.newByteBuffer(151413 * 4 * 3);
      l515PointsOnly = new Mat(1, 151413, opencv_core.CV_32FC3, new BytePointer(l515PointsOnlyHostBuffer));

      openCLManager.create();
      numberOfOusterPoints = 1024 * 128;
      ousterInBytesLength = numberOfOusterPoints * 48;
      zed2InBytesLength = 1280 * 720 * 3;
      fusedOutBytesLength = numberOfOusterPoints * 4 * 10; // X,Y,Z,R,G,B,A,Size,Sin,Cos
      BufferUtils.newByteBuffer(fusedOutBytesLength);
      ousterInGPUBuffer = openCLManager.createBufferObject(ousterInBytesLength);
      zed2InGPUBuffer = openCLManager.createBufferObject(zed2InBytesLength);
      fusedOutGPUBuffer = openCLManager.createBufferObject(fusedOutBytesLength);
      projectZED2ToOusterPointsKernel = openCLManager.loadProgramAndCreateKernel("projectZED2ToOusterPoints");
      openCLManager.setKernelArgument(projectZED2ToOusterPointsKernel, 0, ousterInGPUBuffer);
      openCLManager.setKernelArgument(projectZED2ToOusterPointsKernel, 1, zed2InGPUBuffer);
      openCLManager.setKernelArgument(projectZED2ToOusterPointsKernel, 2, fusedOutGPUBuffer);

      pointCloudRenderer.create(numberOfOusterPoints);
      coloredPointCloudDataHostFloatBuffer = BufferUtils.newFloatBuffer(pointCloudRenderer.getVerticesArray().length);
      zed2InHostBuffer = BufferUtils.newByteBuffer(zed2InBytesLength);
      ousterInHostBuffer = BufferUtils.newByteBuffer(ousterInBytesLength);
   }


   @Override
   public void update()
   {
      super.update();

      PointCloud2 ousterPointCloud2 = latestOusterPointCloud.get();
      PointCloud2 l515PointCloud2 = latestL515PointCloud.get();
      Image zed2Image = latestZED2Image.get();

//      if (ousterPointCloud2 != null && l515PointCloud2 != null && zed2Image != null)
      if (ousterPointCloud2 != null && zed2Image != null)
      {
         long ousterTimestamp = ousterPointCloud2.getHeader().getStamp().totalNsecs();

//         ROSOpenCVTools.backMatWithNettyBuffer(l515WithRGB, l515PointCloud2.getData());
//         opencv_core.mixChannels(l515WithRGB, 4, l515PointsOnly, 3, l515RetainXYZChannels, 3);

         ByteBuffer ousterInHeapBuffer = RosTools.sliceNettyBuffer(ousterPointCloud2.getData());
         ousterInHostBuffer.rewind();
         ousterInHostBuffer.put(ousterInHeapBuffer);
         ousterInHostBuffer.rewind();
         openCLManager.enqueueWriteBuffer(ousterInGPUBuffer, ousterInBytesLength, new BytePointer(ousterInHostBuffer));

         ByteBuffer zed2InHeapBuffer = RosTools.sliceNettyBuffer(zed2Image.getData());
         zed2InHostBuffer.rewind();
         zed2InHostBuffer.put(zed2InHeapBuffer);
         zed2InHostBuffer.rewind();
         openCLManager.enqueueWriteBuffer(zed2InGPUBuffer, zed2InBytesLength, new BytePointer(zed2InHostBuffer));

         openCLManager.execute(projectZED2ToOusterPointsKernel, numberOfOusterPoints);

         // l515 points, make into XYZRGBA and stick at the end
         // should be 1024 * 128 jobs?

         coloredPointCloudDataHostFloatBuffer.rewind();
         openCLManager.enqueueReadBuffer(fusedOutGPUBuffer, fusedOutBytesLength, new FloatPointer(coloredPointCloudDataHostFloatBuffer));

         // floats: X,Y,Z,R,G,B,A,0.01,1.0,0.0
         float[] coloredPointDataArray = pointCloudRenderer.getVerticesArray(); // Copy data to this array
         coloredPointCloudDataHostFloatBuffer.rewind();
         coloredPointCloudDataHostFloatBuffer.get(coloredPointDataArray);
         // TODO: Data techinically getting copied twice extra here. Shouldn't be necessary.
//         pointCloudRenderer.updateMeshFast(l515PointCloud2.getWidth() * l515PointCloud2.getHeight()
//                                           + ousterPointCloud2.getWidth() * ousterPointCloud2.getHeight());
         pointCloudRenderer.updateMeshFast(numberOfOusterPoints);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(RosTools.ZED2_LEFT_EYE_VIDEO);
      zed2ReceivedPlot.render(zed2ReceivedCount);
      ImGui.text(RosTools.L515_POINT_CLOUD);
      l515ReceivedPlot.render(l515ReceivedCount);
      ImGui.text(RosTools.OUSTER_POINT_CLOUD);
      ousterReceivedPlot.render(ousterReceivedCount);
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      zed2LeftEyeSubscriber = new AbstractRosTopicSubscriber<Image>(Image._TYPE)
      {
         @Override
         public void onNewMessage(Image image)
         {
            ++zed2ReceivedCount;
            latestZED2Image.set(image);
         }
      };
      ros1Node.attachSubscriber(RosTools.ZED2_LEFT_EYE_VIDEO, zed2LeftEyeSubscriber);
      l515Subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            ++l515ReceivedCount;
            latestL515PointCloud.set(pointCloud2);
         }
      };
      ros1Node.attachSubscriber(RosTools.L515_POINT_CLOUD, l515Subscriber);
      ousterSubscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud2)
         {
            ++ousterReceivedCount;
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

   public static void main(String[] args)
   {
      OpenCLManager openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLManager.loadProgramAndCreateKernel("projectZED2ToOusterPoints");
   }
}
