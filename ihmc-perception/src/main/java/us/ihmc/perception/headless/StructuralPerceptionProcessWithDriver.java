package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.ouster.OusterDepthExtractionKernel;
import us.ihmc.perception.ouster.OusterNetServer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.function.Supplier;

/**
 * StructuralPerceptionProcessWithDriver is a headless process that runs the perception frontend for structure-specific measurements such as planar regions.
 * color, depth, and point cloud data using the depth data obtained from the structure sensor on the robot. (usually a large FOV sensor, Ouster currently).
 * Structure refers to the surrounding geometric structures such as walls, pillars, ceilings, and other large objects. It may also assist the
 * TerrainPerceptionProcess by publishing additional signals measured over the terrain. This class may be extended in the future to support height map
 * extraction, iterative-closest point based registration, LidarScanMessage publisher, and more.
 * <p>
 * Primary responsibilities include (but are not limited to):
 * 1. Reads depth data from the sensor.
 * 2. Extracts planar regions from the depth data.
 * 3. Publishes compressed depth images on the depth topic
 * 4. Publishes planar regions on the planar regions topic
 */
public class StructuralPerceptionProcessWithDriver
{
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<ImageMessage> ros2DepthImagePublisher;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private final OusterNetServer ouster;
   private int depthWidth;
   private int depthHeight;
   private int numberOfPointsPerFullScan;
   private OusterDepthExtractionKernel depthExtractionKernel = null;
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();

   private final ROS2Helper ros2Helper;
   private ROS2Topic<ImageMessage> depthTopic;
   private ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;

   public StructuralPerceptionProcessWithDriver(ROS2Topic<ImageMessage> depthTopic,
                                                ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic,
                                                Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.depthTopic = depthTopic;
      this.frameRegionsTopic = frameRegionsTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      ouster = new OusterNetServer();
      ouster.start();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "spherical_regions_node");
      ros2Helper = new ROS2Helper(ros2Node);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_node");
      LogTools.info("Publishing ROS 2 depth images: {}", depthTopic);
      ros2DepthImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
      LogTools.info("Spinning Realtime ROS 2 node");
      realtimeROS2Node.spin();
      openCLManager = new OpenCLManager();

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

   private void onFrameReceived()
   {
      LogTools.warn("Depth Extraction Kernel: {}", depthExtractionKernel);

      if (depthExtractionKernel == null)
      {
         LogTools.info("Ouster has been initialized.");
         depthWidth = ouster.getImageWidth();
         depthHeight = ouster.getImageHeight();
         numberOfPointsPerFullScan = depthWidth * depthHeight;
         LogTools.info("Ouster width: {} height: {} # points: {}", depthWidth, depthHeight, numberOfPointsPerFullScan);

         // TODO: Combine rapid region extraction and depth extraction kernels into single program

         depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager);
         compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
         pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
         pngImageBytePointer = new BytePointer(pngImageBuffer);

         ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);

         LogTools.warn("Depth Height: {} Width: {}", depthHeight, depthWidth);

         openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");
         rapidRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, openCLProgram, depthHeight, depthWidth);
         rapidRegionsExtractor.setPatchSizeChanged(false);


         LogTools.warn("ROS2 Property Set Group registered: {}", ros2PropertySetGroup);

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

   private void extractCompressAndPublish()
   {
      ros2PropertySetGroup.update();

      // Important not to store as a field, as update() needs to be called each frame
      ReferenceFrame cameraFrame = sensorFrameUpdater.get();

      depthExtractionKernel.runKernel(new RigidBodyTransform());
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
      ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(outputImageMessage);
      outputImageMessage.setSequenceNumber(sequenceNumber++);
      outputImageMessage.setImageWidth(depthWidth);
      outputImageMessage.setImageHeight(depthHeight);
      ros2DepthImagePublisher.publish(outputImageMessage);

      BytedecoImage depthImage = depthExtractionKernel.getExtractedDepthImage();

      FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();



      extractFramePlanarRegionsList(depthImage, ReferenceFrame.getWorldFrame(), framePlanarRegionsList);
      PlanarRegionsList planarRegionsList = framePlanarRegionsList.getPlanarRegionsList();

      LogTools.info("Extracted {} planar regions", planarRegionsList.getNumberOfPlanarRegions());

//      PerceptionDebugTools.displayDepth("Depth", depthImage.getBytedecoOpenCVMat(), 1);

      PerceptionMessageTools.publishFramePlanarRegionsList(framePlanarRegionsList, frameRegionsTopic, ros2Helper);

      rapidRegionsExtractor.setProcessing(false);


   }

   private void extractFramePlanarRegionsList(BytedecoImage depthImage, ReferenceFrame cameraFrame, FramePlanarRegionsList framePlanarRegionsList)
   {
      rapidRegionsExtractor.update(depthImage, cameraFrame, framePlanarRegionsList);
   }

   public static void main(String[] args)
   {
      new StructuralPerceptionProcessWithDriver(PerceptionAPI.OUSTER_DEPTH_IMAGE, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ReferenceFrame::getWorldFrame);
   }
}
