package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.PlanarRegionsListWithPoseMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.OpenCVImageFormat;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.ouster.OusterDepthExtractionKernel;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class StructuralPerceptionProcessWithDriver
{
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<ImageMessage> ros2DepthImagePublisher;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private final NettyOuster ouster;
   private int depthWidth;
   private int depthHeight;
   private int numberOfPointsPerFullScan;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();

   private final ROS2Helper ros2Helper;
   private ROS2Topic<ImageMessage> depthTopic;
   private ROS2Topic<PlanarRegionsListMessage> regionsTopic;
   private ROS2Topic<PlanarRegionsListWithPoseMessage> regionsWithPoseTopic;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final RapidPlanarRegionsExtractor rapidRegionsExtractor;

   public StructuralPerceptionProcessWithDriver(ROS2Topic<ImageMessage> depthTopic,
                                                ROS2Topic<PlanarRegionsListMessage> regionsTopic,
                                                ROS2Topic<PlanarRegionsListWithPoseMessage> regionsWithPoseTopic,
                                                Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.depthTopic = depthTopic;
      this.regionsTopic = regionsTopic;
      this.regionsWithPoseTopic = regionsWithPoseTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ouster = new NettyOuster();
      ouster.bind();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "spherical_regions_node");
      ros2Helper = new ROS2Helper(ros2Node);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_node");
      LogTools.info("Publishing ROS 2 depth images: {}", depthTopic);
      ros2DepthImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
      LogTools.info("Spinning Realtime ROS 2 node");
      realtimeROS2Node.spin();

      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();

      extractCompressAndPublishThread = MissingThreadTools.newSingleThreadExecutor("CopyAndPublish", true, 1);
      // Using incoming Ouster UDP Netty events as the thread scheduler. Only called on last datagram of frame.
      ouster.setOnFrameReceived(this::onFrameReceived);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         ouster.setOnFrameReceived(null);
                                                         ouster.destroy();
                                                         ThreadTools.sleepSeconds(0.5);
                                                         extractCompressAndPublishThread.destroy();
                                                      }, getClass().getSimpleName() + "Shutdown"));
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
            depthWidth = ouster.getImageWidth();
            depthHeight = ouster.getImageHeight();
            numberOfPointsPerFullScan = depthWidth * depthHeight;
            LogTools.info("Ouster width: {} height: {} # points: {}", depthWidth, depthHeight, numberOfPointsPerFullScan);

            // TODO: Combine rapid region extraction and depth extraction kernels into single program

            openCLProgram = openCLManager.loadProgram("OusterDepthImageExtraction", "RapidRegionsExtractor.cl");

            depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, openCLProgram);
            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
            pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
            pngImageBytePointer = new BytePointer(pngImageBuffer);

            rapidRegionsExtractor.create(openCLManager, openCLProgram, depthWidth, depthHeight);
            rapidRegionsExtractor.setPatchSizeChanged(false);

            ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.SPHERICAL_RAPID_REGION_PARAMETERS, rapidRegionsExtractor.getParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.SPHERICAL_POLYGONIZER_PARAMETERS,
                                                           rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getPolygonizerParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS,
                                                           rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getConcaveHullFactoryParameters());
         }

         // Fast memcopy while the ouster thread is blocked
         depthExtractionKernel.copyLidarFrameBuffer();
         extractCompressAndPublishThread.clearQueueAndExecute(this::extractCompressAndPublish);
      }
   }

   /**
    * Synchronized to make sure it's only running ever once at a time.
    * This should also be guaranteed by the ResettableExceptionHandlingExecutorService.
    */
   private synchronized void extractCompressAndPublish()
   {
      // Important not to store as a field, as update() needs to be called each frame
      ReferenceFrame cameraFrame = sensorFrameUpdater.get();
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      depthExtractionKernel.runKernel();
      // Encode as PNG which is lossless and handles single channel images.
      opencv_imgcodecs.imencode(".png", depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat(), pngImageBytePointer, compressionParameters);

      outputImageMessage.getPosition().set(cameraPose.getPosition());
      outputImageMessage.getOrientation().set(cameraPose.getOrientation());
      MessageTools.toMessage(ouster.getAquisitionInstant(), outputImageMessage.getAcquisitionTime());
      // Sadly, Pub Sub makes us go through a TByteArrayList. If we rewrite our DDS layer, we should allow a memcpy to native DDS buffer.
      outputImageMessage.getData().resetQuick();
      for (int i = 0; i < pngImageBytePointer.limit(); i++)
      {
         outputImageMessage.getData().add(pngImageBytePointer.get(i));
      }
      outputImageMessage.setFormat(OpenCVImageFormat.PNG.ordinal());
      outputImageMessage.setSequenceNumber(sequenceNumber++);
      outputImageMessage.setImageWidth(depthWidth);
      outputImageMessage.setImageHeight(depthHeight);
      ros2DepthImagePublisher.publish(outputImageMessage);

      BytedecoImage depthImage = depthExtractionKernel.getExtractedDepthImage();

      PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
      extractPlanarRegionsListWithPose(depthImage, ReferenceFrame.getWorldFrame(), planarRegionsListWithPose);
      PlanarRegionsList planarRegionsList = planarRegionsListWithPose.getPlanarRegionsList();

      LogTools.info("Extracted {} planar regions", planarRegionsList.getNumberOfPlanarRegions());

      PerceptionTools.publishPlanarRegionsList(planarRegionsList, regionsTopic, ros2Helper);
   }

   private void extractPlanarRegionsListWithPose(BytedecoImage depthImage, ReferenceFrame cameraFrame, PlanarRegionsListWithPose planarRegionsListWithPose)
   {
      rapidRegionsExtractor.update(depthImage, cameraFrame, planarRegionsListWithPose);
   }

   public static void main(String[] args)
   {
      new StructuralPerceptionProcessWithDriver(ROS2Tools.OUSTER_DEPTH_IMAGE,
                                                ROS2Tools.PERSPECTIVE_RAPID_REGIONS,
                                                ROS2Tools.PERSPECTIVE_RAPID_REGIONS_WITH_POSE,
                                                ReferenceFrame::getWorldFrame);
   }
}
