package us.ihmc.rdx.perception;

import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opticalFlow.OpenCVOpticalFlowProcessor;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

public class RDXOpticalFlowDemo
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "optical_flow_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;
   private OpenCVOpticalFlowProcessor opticalFlow;

   private final RDXBaseUI baseUI = new RDXBaseUI("Optical Flow Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();

   public RDXOpticalFlowDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      imageRetriever.start();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");
   }

   private void processImage()
   {
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);
      RawImage depthImage = imageRetriever.getLatestRawDepthImage();

      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);
      imagePublisher.setNextGpuDepthImage(depthImage.get());

      opticalFlow.setNewImage(colorImage);
      try (GpuMat flowMat = opticalFlow.calculateFlow()) {
         Mat cpuFlowMat = new Mat();
         flowMat.download(cpuFlowMat);
         PerceptionDebugTools.display("flow", cpuFlowMat, 1);
      }

      colorImage.release();
      depthImage.release();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            perceptionVisualizerPanel // TODO: Finish
         }

      });
   }
}
