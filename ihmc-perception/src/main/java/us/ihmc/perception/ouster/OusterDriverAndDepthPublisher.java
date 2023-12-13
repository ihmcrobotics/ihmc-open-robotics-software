package us.ihmc.perception.ouster;

import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.communication.ros2.ROS2HeartbeatMonitor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.steppableRegions.RemoteSteppableRegionsUpdater;
import us.ihmc.perception.steppableRegions.SteppableRegionsAPI;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterDriverAndDepthPublisher
{
   private final ROS2HeartbeatMonitor publishLidarScanMonitor;
   private final ROS2HeartbeatMonitor publishSteppableRegionsMonitor;
   private final ROS2HeartbeatMonitor publishHeightMapMonitor;
   private final Supplier<HumanoidReferenceFrames> humanoidReferenceFramesSupplier;
   private final Runnable asynchronousCompressAndPublish = this::asynchronousCompressAndPublish;
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private final OusterNetServer ouster;
   private final OusterDepthPublisher depthPublisher;
   private final OusterHeightMapUpdater heightMapUpdater;
   private final RemoteSteppableRegionsUpdater steppableRegionsUpdater;
   private OpenCLManager openCLManager;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private volatile HumanoidReferenceFrames humanoidReferenceFrames;
   private final MutableReferenceFrame ousterSensorFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());

   public OusterDriverAndDepthPublisher(ROS2ControllerPublishSubscribeAPI ros2,
                                        Supplier<HumanoidReferenceFrames> humanoidReferenceFramesSupplier,
                                        ROS2Topic<ImageMessage> imageMessageTopic,
                                        ROS2Topic<LidarScanMessage> lidarScanTopic)
   {
      this.humanoidReferenceFramesSupplier = humanoidReferenceFramesSupplier;

      publishLidarScanMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_LIDAR_SCAN);
      publishSteppableRegionsMonitor = new ROS2HeartbeatMonitor(ros2, SteppableRegionsAPI.PUBLISH_STEPPABLE_REGIONS);
      publishHeightMapMonitor = new ROS2HeartbeatMonitor(ros2, PerceptionAPI.REQUEST_HEIGHT_MAP);

      ouster = new OusterNetServer();
      ouster.start();

      depthPublisher = new OusterDepthPublisher(imageMessageTopic, lidarScanTopic, publishLidarScanMonitor::isAlive);
      heightMapUpdater = new OusterHeightMapUpdater(ros2);

      steppableRegionsUpdater = new RemoteSteppableRegionsUpdater(ros2, new SteppableRegionCalculatorParameters(), publishSteppableRegionsMonitor::isAlive);
      heightMapUpdater.attachHeightMapConsumer(steppableRegionsUpdater::submitLatestHeightMapMessage);
      steppableRegionsUpdater.start();

      if (publishHeightMapMonitor.isAlive())
         heightMapUpdater.start();

      publishHeightMapMonitor.setAlivenessChangedCallback(isAlive ->
      {
         if (isAlive)
            heightMapUpdater.start();
         else
            heightMapUpdater.stop();
      });

      extractCompressAndPublishThread = MissingThreadTools.newSingleThreadExecutor("CopyAndPublish", true, 1);
      // Using incoming Ouster UDP Netty events as the thread scheduler. Only called on last datagram of frame.
      ouster.setOnFrameReceived(this::onFrameReceived);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         ouster.setOnFrameReceived(null);
         ouster.destroy();

         publishLidarScanMonitor.destroy();
         publishHeightMapMonitor.destroy();
         depthPublisher.destroy();
         heightMapUpdater.stop();
         heightMapUpdater.destroy();

         extractCompressAndPublishThread.destroy();

         System.out.println("Ouster driver/publisher shutting down...");
      }, getClass().getSimpleName() + "Shutdown"));
   }

   // If we aren't doing anything, copy the data and publish it.
   private synchronized void onFrameReceived()
   {
      if (ouster.isInitialized())
      {
         if (openCLManager == null)
         {
            openCLManager = new OpenCLManager();
            depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, publishLidarScanMonitor::isAlive, publishHeightMapMonitor::isAlive);
            depthPublisher.initialize(ouster.getImageWidth(), ouster.getImageHeight());
         }

         synchronized (this)
         {
            humanoidReferenceFrames = humanoidReferenceFramesSupplier.get();
         }

         // Fast memcopy while the ouster thread is blocked
         depthExtractionKernel.copyLidarFrameBuffer();
         extractCompressAndPublishThread.clearQueueAndExecute(asynchronousCompressAndPublish);

         if (publishHeightMapMonitor.isAlive())
         {
            heightMapUpdater.updateWithDataBuffer(humanoidReferenceFrames.getMidFeetZUpFrame(),
                                                  ousterSensorFrame.getReferenceFrame(),
                                                  depthExtractionKernel.getPointCloudInWorldFrame(),
                                                  ouster.getImageHeight() * ouster.getImageWidth(),
                                                  ouster.getAquisitionInstant());
         }
      }
   }

   private void asynchronousCompressAndPublish()
   {
      synchronized (this) // Avoiding concurrent modification of transforms
      {
         humanoidReferenceFrames.getOusterLidarFrame().getTransformToDesiredFrame(ousterSensorFrame.getTransformToParent(), ReferenceFrame.getWorldFrame());
         ousterSensorFrame.getReferenceFrame().update();
      }
      depthPublisher.extractCompressAndPublish(ousterSensorFrame.getReferenceFrame(),
                                               depthExtractionKernel,
                                               ouster.getAquisitionInstant(),
                                               ouster.getBeamAltitudeAnglesBuffer(),
                                               ouster.getBeamAzimuthAnglesBuffer());
   }
}
