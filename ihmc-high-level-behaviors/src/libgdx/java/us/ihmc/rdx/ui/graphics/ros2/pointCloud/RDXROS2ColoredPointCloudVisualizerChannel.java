package us.ihmc.rdx.ui.graphics.ros2.pointCloud;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.ouster.OusterNetServer;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.SwapReference;

import java.nio.ByteBuffer;

/**
 * The common part of the depth and color channels.
 * A channel is image data coming in from ROS 2 and making it's
 * way into a kernel for point cloud rendering.
 */
public abstract class RDXROS2ColoredPointCloudVisualizerChannel
{
   protected final ImageMessageDecoder imageMessageDecoder = new ImageMessageDecoder();
   protected final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private ROS2Topic<ImageMessage> topic;
   private ROS2Subscription<ImageMessage> imageMessageSubscription;
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();
   protected final SwapReference<ImageMessage> imageMessageSwapReference = new SwapReference<>(ImageMessage::new);
   private final ResettableExceptionHandlingExecutorService channelDecompressionThreadExecutor;
   private final Runnable decompressionAsynchronousThread = this::decompressionAsynchronousThread;
   private boolean initialized = false;
   private final Notification subscribedImageAvailable = new Notification();
   private final Notification decompressedImageReady = new Notification();
   private volatile boolean receivedOne = false;
   protected int imageWidth;
   protected int imageHeight;
   private int totalNumberOfPixels = 0;
   private float fx;
   private float fy;
   private float cx;
   private float cy;
   private final RotationMatrix rotationMatrixToWorld = new RotationMatrix();
   private final Vector3D translationToWorld = new Vector3D();
   private float depthDiscretization;
   private CameraModel cameraModel;
   private final ByteBuffer ousterBeamAltitudeAnglesBuffer = NativeMemoryTools.allocate(Float.BYTES * OusterNetServer.MAX_POINTS_PER_COLUMN);
   private final ByteBuffer ousterBeamAzimuthAnglesBuffer = NativeMemoryTools.allocate(Float.BYTES * OusterNetServer.MAX_POINTS_PER_COLUMN);

   private ImGuiAveragedFrequencyText frequencyText = new ImGuiAveragedFrequencyText();

   public RDXROS2ColoredPointCloudVisualizerChannel(String name, ROS2Topic<ImageMessage> topic)
   {
      this.topic = topic;

//      delayPlot = new ImPlotDoublePlot(name + " Delay", 30);

      boolean daemon = true;
      int queueSize = 1;
      channelDecompressionThreadExecutor = MissingThreadTools.newSingleThreadExecutor("ChannelDecompressionThread", daemon, queueSize);
   }

   public void subscribe(ROS2Node ros2Node, Object imageMessagesSyncObject)
   {
      if (imageMessageSubscription != null)
         unsubscribe();

      imageMessageSubscription = ros2Node.createSubscription(topic, subscriber -> {
         synchronized (imageMessagesSyncObject)
         {
            imageMessage.getData().resetQuick();
            subscriber.takeNextData(imageMessage, sampleInfo);
            subscribedImageAvailable.set();
            receivedOne = true;
            frequencyText.ping();
         }
      });
   }

   public void unsubscribe()
   {
      if (imageMessageSubscription != null)
      {
         imageMessageSubscription.remove();
         imageMessageSubscription = null;
      }
   }

   /**
    * This method is synchronized so that the ROS 2 callback is not updating the ImageMessage
    * while this update is running. We store everything as fields and the data in a swap
    * reference so we can do the decompression and run the GPU kernels without needing to
    * synchronize that part.
    */
   public void update(OpenCLManager openCLManager)
   {
      if (subscribedImageAvailable.poll())
      {
         if (!initialized)
         {
            imageWidth = imageMessage.getImageWidth();
            imageHeight = imageMessage.getImageHeight();
            totalNumberOfPixels = imageHeight * imageWidth;
            imageMessageSwapReference.initializeBoth(message -> message.set(imageMessage));

            initialize(openCLManager);
            initialized = true;
         }

         fx = imageMessage.getFocalLengthXPixels();
         fy = imageMessage.getFocalLengthYPixels();
         cx = imageMessage.getPrincipalPointXPixels();
         cy = imageMessage.getPrincipalPointYPixels();
         depthDiscretization = imageMessage.getDepthDiscretization();
         translationToWorld.set(imageMessage.getPosition());
         rotationMatrixToWorld.set(imageMessage.getOrientation());

         // We decompress outside of the incoming message synchronization block.
         // Using an internal swap reference to the unpacked data, we can allow ROS 2
         // to not have to wait for compression to finish and also not have to copy
         // the unpacked result to a decompression input buffer.
         imageMessageSwapReference.getForThreadOne().set(imageMessage);
         imageMessageSwapReference.swap();
         // FIXME: This call prints "no afterExecute handlers" warnings whe the Ouster Fisheye point cloud's refresh rate is too fast
         channelDecompressionThreadExecutor.clearQueueAndExecute(decompressionAsynchronousThread);

         messageSizeReadout.update(imageMessage.getData().size());
         sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());
//         delayPlot.addValue(MessageTools.calculateDelay(imageMessage));

         cameraModel = CameraModel.getCameraModel(imageMessage);
         MessageTools.extractIDLSequence(imageMessage.getOusterBeamAltitudeAngles(), ousterBeamAltitudeAnglesBuffer);
         MessageTools.extractIDLSequence(imageMessage.getOusterBeamAzimuthAngles(), ousterBeamAzimuthAnglesBuffer);
      }
   }

   private void decompressionAsynchronousThread()
   {
      decompress();
      decompressedImageReady.set();
   }

   protected abstract void initialize(OpenCLManager openCLManager);

   protected abstract void decompress();

   protected abstract Object getDecompressionAccessSyncObject();

   public void destroy()
   {
      channelDecompressionThreadExecutor.destroy();
      imageMessageDecoder.destroy();
   }

   public ROS2Topic<ImageMessage> getTopic()
   {
      return topic;
   }

   public Notification getDecompressedImageReady()
   {
      return decompressedImageReady;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getTotalNumberOfPixels()
   {
      return totalNumberOfPixels;
   }

   public float getFx()
   {
      return fx;
   }

   public float getFy()
   {
      return fy;
   }

   public float getCx()
   {
      return cx;
   }

   public float getCy()
   {
      return cy;
   }

   public Vector3D getTranslationToWorld()
   {
      return translationToWorld;
   }

   public RotationMatrix getRotationMatrixToWorld()
   {
      return rotationMatrixToWorld;
   }

   public boolean getReceivedOne()
   {
      return receivedOne;
   }

   public RDXMessageSizeReadout getMessageSizeReadout()
   {
      return messageSizeReadout;
   }

   public RDXSequenceDiscontinuityPlot getSequenceDiscontinuityPlot()
   {
      return sequenceDiscontinuityPlot;
   }

   public float getDepthDiscretization()
   {
      return depthDiscretization;
   }

   public CameraModel getCameraModel()
   {
      return cameraModel;
   }

   public ByteBuffer getOusterBeamAltitudeAnglesBuffer()
   {
      return ousterBeamAltitudeAnglesBuffer;
   }

   public ByteBuffer getOusterBeamAzimuthAnglesBuffer()
   {
      return ousterBeamAzimuthAnglesBuffer;
   }

   public void setTopic(ROS2Topic<ImageMessage> topic)
   {
      this.topic = topic;
   }

   public ImGuiAveragedFrequencyText getFrequencyText()
   {
      return frequencyText;
   }
}
