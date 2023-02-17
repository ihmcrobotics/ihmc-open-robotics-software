package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.tools.ImageMessageDecompressionInput;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.ui.graphics.RDXMessageSizeReadout;
import us.ihmc.rdx.ui.graphics.RDXSequenceDiscontinuityPlot;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public abstract class RDXROS2ColoredPointCloudVisualizerChannel
{
   protected final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final ROS2Topic<ImageMessage> topic;
   private final ImPlotFrequencyPlot frequencyPlot;
   private final ImPlotDoublePlot delayPlot;
   private final RDXMessageSizeReadout messageSizeReadout = new RDXMessageSizeReadout();
   private final RDXSequenceDiscontinuityPlot sequenceDiscontinuityPlot = new RDXSequenceDiscontinuityPlot();
   protected final ImageMessageDecompressionInput decompressionInput = new ImageMessageDecompressionInput();
   private boolean initialized = false;
   private volatile boolean imageAvailable = false;
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
            imageAvailable = true;
            receivedOne = true;
         }
      });
   }

   public void update(OpenCLManager openCLManager)
   {
      if (!initialized)
      {
         imageWidth = imageMessage.getImageWidth();
         imageHeight = imageMessage.getImageHeight();
         totalNumberOfPixels = imageHeight * imageWidth;
         int bytesPerPixel = ImageMessageFormat.getFormat(imageMessage).getBytesPerPixel();
         decompressionInput.setup(totalNumberOfPixels * bytesPerPixel);

         initialize(openCLManager);
         initialized = true;
      }

      fx = imageMessage.getFocalLengthXPixels();
      fy = imageMessage.getFocalLengthYPixels();
      cx = imageMessage.getPrincipalPointXPixels();
      cy = imageMessage.getPrincipalPointYPixels();
      translationToWorld.set(imageMessage.getPosition());
      rotationMatrixToWorld.set(imageMessage.getOrientation());

      decompressionInput.extract(imageMessage);

      messageSizeReadout.update(imageMessage.getData().size());
      sequenceDiscontinuityPlot.update(imageMessage.getSequenceNumber());
      delayPlot.addValue(PerceptionMessageTools.calculateDelay(imageMessage));

      imageAvailable = false;
   }

   protected abstract void initialize(OpenCLManager openCLManager);

   public ROS2Topic<ImageMessage> getTopic()
   {
      return topic;
   }

   public boolean getImageAvailable()
   {
      return imageAvailable;
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
}
