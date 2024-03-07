package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opticalFlow.OpenCVDenseOpticalFlowProcessor;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

public class RDXDenseOpticalFlowDemo
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "optical_flow_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;
   private OpenCVDenseOpticalFlowProcessor opticalFlow;

   private final RDXBaseUI baseUI = new RDXBaseUI("Optical Flow Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();

   private boolean done = false;

   public RDXDenseOpticalFlowDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      imageRetriever.start();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (!done)
      {
         processImage();
      }

      destroy();
   }

   private void processImage()
   {
      RawImage depthImage = imageRetriever.getLatestRawDepthImage();
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      if (opticalFlow == null)
         opticalFlow = new OpenCVDenseOpticalFlowProcessor(colorImage);

      opticalFlow.setNewImage(colorImage);
      try (GpuMat flowMat = opticalFlow.calculateFlow();
           Mat angle = new Mat();
           Mat magnitude = new Mat();
           Mat hsv = new Mat();
           Mat bgr = new Mat()) {

         if (flowMat.isNull())
            System.out.println("IM NULL");

         Mat cpuFlowMat = new Mat();
         flowMat.download(cpuFlowMat);
         MatVector flowParts = new MatVector();
         opencv_core.split(cpuFlowMat, flowParts);

         opencv_core.cartToPolar(flowParts.get(0), flowParts.get(1), magnitude, angle, true);
         opencv_core.normalize(magnitude, magnitude, 0.0, 1.0, opencv_core.NORM_MINMAX, -1, opencv_core.noArray());
         float factor = (float) ((1.0/360.0)*(180.0/255.0));
         opencv_core.multiply(angle, new Mat(angle.size(), angle.type(), new Scalar(factor)), angle);

         MatVector hsvBuild = new MatVector();
         hsvBuild.push_back(angle);
         hsvBuild.push_back(Mat.ones(angle.size(), opencv_core.CV_32F).asMat());
         hsvBuild.push_back(magnitude);
         opencv_core.merge(hsvBuild, hsv);
         hsv.convertTo(hsv, opencv_core.CV_8U, 255.0, 0);
         opencv_imgproc.cvtColor(hsv, bgr, opencv_imgproc.COLOR_HSV2BGR);

         RawImage bgrImage = new RawImage(colorImage.getSequenceNumber(),
                                          colorImage.getAcquisitionTime(),
                                          colorImage.getImageWidth(),
                                          colorImage.getImageHeight(),
                                          colorImage.getDepthDiscretization(),
                                          bgr,
                                          null,
                                          bgr.type(),
                                          colorImage.getFocalLengthX(),
                                          colorImage.getFocalLengthY(),
                                          colorImage.getPrincipalPointX(),
                                          colorImage.getPrincipalPointY(),
                                          colorImage.getPosition(),
                                          colorImage.getOrientation());

         bgrImage.getGpuImageMat();
         imagePublisher.setNextColorImage(bgrImage.get(), RobotSide.RIGHT);

         cpuFlowMat.release();
         bgrImage.release();
         flowParts.releaseReference();
         hsvBuild.releaseReference();
      }

      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);
      imagePublisher.setNextGpuDepthImage(depthImage.get());

      colorImage.release();
      depthImage.release();
   }

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
      opticalFlow.destroy();
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            RDXROS2ImageMessageVisualizer depthImageVisualizer = new RDXROS2ImageMessageVisualizer("Depth Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_DEPTH);
            perceptionVisualizerPanel.addVisualizer(depthImageVisualizer, PerceptionAPI.REQUEST_ZED_DEPTH);

            RDXROS2ImageMessageVisualizer opticalFlowImageVisualizer = new RDXROS2ImageMessageVisualizer("Optical Flow",
                                                                                                         PubSubImplementation.FAST_RTPS,
                                                                                                         PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            perceptionVisualizerPanel.addVisualizer(opticalFlowImageVisualizer, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            baseUI.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizerPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            done = true;
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXDenseOpticalFlowDemo();
   }
}
