package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.memory.NativeMemoryTools;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicReference;

public class RDXROS2OusterPointCloudVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private IHMCROS2Callback<ImageMessage> ros2Callback = null;
   private final ImPlotFrequencyPlot frequencyPlot = new ImPlotFrequencyPlot("Hz", 30);
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Delay", 30);
   private final ImFloat pointSize = new ImFloat(0.01f);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean subscribed = new ImBoolean(true);
   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
   private int totalNumberOfPoints;
   private final AtomicReference<ImageMessage> imageMessageReference = new AtomicReference<>(null);
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private ByteBuffer decompressionInputBuffer;
   private BytePointer decompressionInputBytePointer;
   private Mat decompressionInputMat;
   private BytedecoImage decompressionOutputImage;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private final OpenCLFloatParameters parametersOpenCLFloatBuffer = new OpenCLFloatParameters();
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();
   private int depthWidth;
   private int depthHeight;
   private final RigidBodyTransform sensorTransformToWorld = new RigidBodyTransform();

   public RDXROS2OusterPointCloudVisualizer(String title, ROS2Node ros2Node, ROS2Topic<ImageMessage> topic)
   {
      super(title + " (ROS 2)");
      this.ros2Node = ros2Node;
      this.topic = topic;

      setSubscribed(subscribed.get());
   }

   private void subscribe()
   {
      subscribed.set(true);
      ros2Callback = new IHMCROS2Callback<>(ros2Node,
                                            topic.withType(ImageMessage.class),
                                            ROS2QosProfile.BEST_EFFORT(),
                                            this::queueRenderImageBasedPointCloud);
   }

   private void queueRenderImageBasedPointCloud(ImageMessage message)
   {
      frequencyPlot.ping();
      // TODO: Possibly decompress on a thread here
      // TODO: threadQueue.clearQueueAndExecute(() ->
      imageMessageReference.set(message);
   }

   @Override
   public void create()
   {
      super.create();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");
   }

   @Override
   public void update()
   {
      super.update();

      if (subscribed.get() && isActive())
      {
         ImageMessage imageMessage = imageMessageReference.getAndSet(null);
         if (imageMessage != null)
         {
            if (decompressionInputBuffer == null)
            {
               depthWidth = imageMessage.getImageWidth();
               depthHeight = imageMessage.getImageHeight();
               totalNumberOfPoints = depthWidth * depthHeight;
               pointCloudRenderer.create(totalNumberOfPoints);
               decompressionInputBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
               decompressionInputBytePointer = new BytePointer(decompressionInputBuffer);
               decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
               decompressionOutputImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
               decompressionOutputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
               pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX,
                                                              pointCloudRenderer.getVertexBuffer());
               pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
               LogTools.info("Allocated new buffers. {} points.", totalNumberOfPoints);
            }

            sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());

            int numberOfBytes = imageMessage.getData().size();
            decompressionInputBuffer.rewind();
            decompressionInputBuffer.limit(depthWidth * depthHeight * 2);
            for (int i = 0; i < numberOfBytes; i++)
            {
               decompressionInputBuffer.put(imageMessage.getData().get(i));
            }
            decompressionInputBuffer.flip();

            messageSizeReadout.update(numberOfBytes);

            decompressionInputBytePointer.position(0);
            decompressionInputBytePointer.limit(numberOfBytes);

            decompressionInputMat.cols(numberOfBytes);
            decompressionInputMat.data(decompressionInputBytePointer);

            decompressionOutputImage.getBackingDirectByteBuffer().rewind();
            opencv_imgcodecs.imdecode(decompressionInputMat,
                                      opencv_imgcodecs.IMREAD_UNCHANGED,
                                      decompressionOutputImage.getBytedecoOpenCVMat());
            decompressionOutputImage.getBackingDirectByteBuffer().rewind();

            // TODO: Create tuners for these
            double verticalFieldOfView = Math.PI / 2.0;
            double horizontalFieldOfView = 2.0 * Math.PI;

            parametersOpenCLFloatBuffer.setParameter((float) horizontalFieldOfView);
            parametersOpenCLFloatBuffer.setParameter((float) verticalFieldOfView);
            parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
            parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
            parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM00());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM01());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM02());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM10());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM11());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM12());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM20());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM21());
            parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM22());
            parametersOpenCLFloatBuffer.setParameter(depthWidth);
            parametersOpenCLFloatBuffer.setParameter(depthHeight);
            parametersOpenCLFloatBuffer.setParameter(pointSize.get());

            parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
            decompressionOutputImage.writeOpenCLImage(openCLManager);
            pointCloudRenderer.updateMeshFastestBeforeKernel();
            pointCloudVertexBuffer.syncWithBackingBuffer();

            openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressionOutputImage.getOpenCLImageObject());
            openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
            openCLManager.execute2D(unpackPointCloudKernel, depthWidth, depthHeight);
            pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

            pointCloudRenderer.updateMeshFastestAfterKernel();

            delayPlot.addValue(TimeTools.calculateDelay(imageMessage.getAcquisitionTimeSecondsSinceEpoch(), imageMessage.getAcquisitionTimeAdditionalNanos()));
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.getHidden(getTitle() + "Subscribed"), subscribed))
      {
         setSubscribed(subscribed.get());
      }
      ImGuiTools.previousWidgetTooltip("Subscribed");
      ImGui.sameLine();
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      if (frequencyPlot.anyPingsYet())
      {
         messageSizeReadout.renderImGuiWidgets();
      }
      ImGui.sliderFloat(labels.get("Point size"), pointSize.getData(), 0.0005f, 0.05f);
      if (frequencyPlot.anyPingsYet())
      {
         frequencyPlot.renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
         sequenceDiscontinuityPlot.renderImGuiWidgets();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
         pointCloudRenderer.getRenderables(renderables, pool);
   }

   public void setSubscribed(boolean subscribed)
   {
      if (subscribed && ros2Callback == null)
      {
         subscribe();
      }
      else if (!subscribed && ros2Callback != null)
      {
         unsubscribe();
      }
   }

   private void unsubscribe()
   {
      subscribed.set(false);
      ros2Callback.destroy();
      ros2Callback = null;
   }

   public boolean isSubscribed()
   {
      return subscribed.get();
   }
}
