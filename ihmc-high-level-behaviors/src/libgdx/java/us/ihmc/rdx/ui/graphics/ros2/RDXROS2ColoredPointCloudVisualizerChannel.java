package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.tools.ImageMessageDecompressionInput;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.SwapReference;

import java.nio.ByteBuffer;

/**
 * The common part of the depth and color channels.
 * A channel is image data coming in from ROS 2 and making it's
 * way into a kernel for point cloud rendering.
 */
public abstract class RDXROS2ColoredPointCloudVisualizerChannel
{
   protected final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final ROS2Topic<ImageMessage> topic;
   private final ImPlotFrequencyPlot frequencyPlot;
   private final ImPlotDoublePlot delayPlot;
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();
   protected final SwapReference<ImageMessageDecompressionInput> decompressionInputSwapReference = new SwapReference<>(ImageMessageDecompressionInput::new);
   private boolean initialized = false;
   protected final Notification subscribedImageAvailable = new Notification();
   protected final Notification readyForDecompression = new Notification();
   protected final Notification decompressedImageReady = new Notification();
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
   private final ByteBuffer ousterPixelShiftsBuffer = NativeMemoryTools.allocate(Integer.BYTES * NettyOuster.MAX_POINTS_PER_COLUMN);
   private final ByteBuffer ousterBeamAltitudeAnglesBuffer = NativeMemoryTools.allocate(Float.BYTES * NettyOuster.MAX_POINTS_PER_COLUMN);
   private final ByteBuffer ousterBeamAzimuthAnglesBuffer = NativeMemoryTools.allocate(Float.BYTES * NettyOuster.MAX_POINTS_PER_COLUMN);

   public RDXROS2ColoredPointCloudVisualizerChannel(String name, ROS2Topic<ImageMessage> topic)
   {
      this.topic = topic;

      frequencyPlot = new ImPlotFrequencyPlot(name + " Hz", 30);
      delayPlot = new ImPlotDoublePlot(name + " Delay", 30);
   }

   public void subscribe(RealtimeROS2Node realtimeROS2Node, Object imageMessagesSyncObject)
   {
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
         synchronized (imageMessagesSyncObject)
         {
            frequencyPlot.ping();
            imageMessage.getData().resetQuick();
            subscriber.takeNextData(imageMessage, sampleInfo);
            subscribedImageAvailable.set();
            receivedOne = true;
         }
      });
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
            int bytesPerPixel = ImageMessageFormat.getFormat(imageMessage).getBytesPerPixel();
            decompressionInputSwapReference.initializeBoth(decompressionInput -> decompressionInput.setup(totalNumberOfPixels * bytesPerPixel));

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

         decompressionInputSwapReference.getForThreadOne().extract(imageMessage);
         decompressionInputSwapReference.swap();
         readyForDecompression.set();

         messageSizeReadout.update(imageMessage.getData().size());
         sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());
         delayPlot.addValue(MessageTools.calculateDelay(imageMessage));

         cameraModel = CameraModel.getCameraModel(imageMessage);
         MessageTools.extractIDLSequenceCastingBytesToInts(imageMessage.getOusterPixelShifts(), ousterPixelShiftsBuffer);
         MessageTools.extractIDLSequence(imageMessage.getOusterBeamAltitudeAngles(), ousterBeamAltitudeAnglesBuffer);
         MessageTools.extractIDLSequence(imageMessage.getOusterBeamAzimuthAngles(), ousterBeamAzimuthAnglesBuffer);
      }
   }

   protected abstract void initialize(OpenCLManager openCLManager);

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

   public ImPlotFrequencyPlot getFrequencyPlot()
   {
      return frequencyPlot;
   }

   public ImPlotDoublePlot getDelayPlot()
   {
      return delayPlot;
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

   public ByteBuffer getOusterPixelShiftsBuffer()
   {
      return ousterPixelShiftsBuffer;
   }

   public ByteBuffer getOusterBeamAltitudeAnglesBuffer()
   {
      return ousterBeamAltitudeAnglesBuffer;
   }

   public ByteBuffer getOusterBeamAzimuthAnglesBuffer()
   {
      return ousterBeamAzimuthAnglesBuffer;
   }
}
