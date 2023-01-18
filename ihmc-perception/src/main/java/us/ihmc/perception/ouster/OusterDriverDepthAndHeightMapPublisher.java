package us.ihmc.perception.ouster;

import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image, possibly an LZ4 compressed LidarScanMessage, and a height map from the Ouster as fast as the frames come in.
 */
public class OusterDriverDepthAndHeightMapPublisher
{
   private final Activator nativesLoadedActivator;

   private final RealtimeROS2Node realtimeROS2Node;
   private final ROS2Topic<?>[] outputTopics;
   private final List<Class<?>> outputTopicsTypes = new ArrayList<>();
   private final HashMap<ROS2Topic<?>, IHMCRealtimeROS2Publisher> publisherMap = new HashMap<>();

   private final FrameUpdater frameUpdater;
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private final NettyOuster ouster;
   private final OusterHeightMapUpdater heightMapUpdater;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private OusterDepthPublisher depthPublisher;
   private OpenCLManager openCLManager;

   public OusterDriverDepthAndHeightMapPublisher(String robotName,
                                                 RealtimeROS2Node realtimeROS2Node,
                                                 FrameUpdater frameUpdater,
                                                 ROS2Topic<?>... outputTopics)
   {
      this.frameUpdater = frameUpdater;
      this.outputTopics = outputTopics;
      this.realtimeROS2Node = realtimeROS2Node;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      heightMapUpdater = new OusterHeightMapUpdater(robotName, realtimeROS2Node);
      ouster = new NettyOuster();
      ouster.bind();

      for (ROS2Topic<?> outputTopic : outputTopics)
      {
         outputTopicsTypes.add(outputTopic.getType());
         LogTools.info("Publishing ROS 2 depth images: {}", outputTopic);
         publisherMap.put(outputTopic, ROS2Tools.createPublisher(realtimeROS2Node, outputTopic, ROS2QosProfile.BEST_EFFORT()));
      }
      // We have to make sure to add this so that the point cloud gets calculated.
      if (!outputTopicsTypes.contains(LidarScanMessage.class))
         outputTopicsTypes.add(LidarScanMessage.class);


      extractCompressAndPublishThread = MissingThreadTools.newSingleThreadExecutor("CopyAndPublish", true, 1);
      // Using incoming Ouster UDP Netty events as the thread scheduler. Only called on last datagram of frame.
      ouster.setOnFrameReceived(this::onFrameReceived);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         ouster.setOnFrameReceived(null);
                                                         ouster.destroy();
                                                         realtimeROS2Node.destroy();
                                                         ThreadTools.sleepSeconds(0.5);
                                                         extractCompressAndPublishThread.destroy();
                                                         heightMapUpdater.stop();
                                                      }, getClass().getSimpleName() + "Shutdown"));
   }

   public void start()
   {
      LogTools.info("Spinning Realtime ROS 2 node");
      realtimeROS2Node.spin();
   }

   // If we aren't doing anything, copy the data and publish it.
   private synchronized void onFrameReceived()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            openCLManager = new OpenCLManager();
            openCLManager.create();
         }

         if (depthExtractionKernel == null)
         {
            LogTools.info("Ouster has been initialized.");
            int depthWidth = ouster.getImageWidth();
            int depthHeight = ouster.getImageHeight();
            int numberOfPointsPerFullScan = depthWidth * depthHeight;
            LogTools.info("Ouster width: {} height: {} # points: {}", depthWidth, depthHeight, numberOfPointsPerFullScan);
            depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, outputTopicsTypes);
            depthPublisher = new OusterDepthPublisher(publisherMap, depthWidth, depthHeight, outputTopics);
         }

         frameUpdater.updateFrames();

         // Fast memcopy while the ouster thread is blocked
         depthExtractionKernel.copyLidarFrameBuffer();
         extractCompressAndPublishThread.clearQueueAndExecute(() -> depthPublisher.extractCompressAndPublish(frameUpdater.getSensorFrame(),
                                                                                                             depthExtractionKernel,
                                                                                                             ouster.getAquisitionInstant()));
         heightMapUpdater.updateWithDataBuffer(frameUpdater.getSensorFrame(),
                                               frameUpdater.getGroundFrame(),
                                               depthExtractionKernel.getPointCloudInSensorFrame(),
                                               ouster.getImageHeight() * ouster.getImageWidth(),
                                               ouster.getAquisitionInstant());
      }
   }


   public interface FrameUpdater
   {
      void updateFrames();

      ReferenceFrame getSensorFrame();

      ReferenceFrame getGroundFrame();
   }

   public static void main(String[] args)
   {
      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_node");
      FrameUpdater frameUpdater = new FrameUpdater()
      {
         @Override
         public void updateFrames()
         {

         }

         @Override
         public ReferenceFrame getSensorFrame()
         {
            return ReferenceFrame.getWorldFrame();
         }

         @Override
         public ReferenceFrame getGroundFrame()
         {
            return ReferenceFrame.getWorldFrame();
         }
      };
      new OusterDriverDepthAndHeightMapPublisher("", realtimeROS2Node, frameUpdater, ROS2Tools.OUSTER_DEPTH_IMAGE);
   }
}
