package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImFloat;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Point2f;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BallDetector;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

public class RDXBallTrackingDemo
{
   private final ROS2Node ros2Node;
   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;

   private final RDXBaseUI baseUI = new RDXBaseUI("Ball Tracker Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();

   private final BallDetector ballDetector = new BallDetector();
   private final Point2f ballCenter = new Point2f(-1.0f, -1.0f);
   private final ImFloat[] hsvLowerBound = {new ImFloat(0.0f), new ImFloat(0.0f), new ImFloat(0.0f)};
   private final ImFloat[] hsvUpperBound = {new ImFloat(255.0f), new ImFloat(255.0f), new ImFloat(255.0f)};

   private boolean done = false;

   public RDXBallTrackingDemo()
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "ball_tracker_demo");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (!done)
         run();

      destroy();
   }

   private void run()
   {
      ballDetector.setHSVLowerBound(hsvLowerBound[0].get(), hsvLowerBound[1].get(), hsvLowerBound[2].get());
      ballDetector.setHSVUpperBound(hsvUpperBound[0].get(), hsvUpperBound[1].get(), hsvUpperBound[2].get());

      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      float radius = ballDetector.detect(colorImage, ballCenter);
      if (radius > 0.0f && ballCenter.x() > 0.0f && ballCenter.y() > 0.0f)
      {
         opencv_imgproc.circle(colorImage.getCpuImageMat(),
                               new Point(Math.round(ballCenter.x()), Math.round(ballCenter.y())),
                               Math.round(radius),
                               new Scalar(0.0, 0.0, 255.0, 255.0));
         colorImage.getGpuImageMat().upload(colorImage.getCpuImageMat());
      }

      imagePublisher.setNextColorImage(colorImage, RobotSide.LEFT);
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
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizerPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            ImGui.text("HSV Lower Bound");
            ImGui.sliderFloat("Hl", hsvLowerBound[0].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Sl", hsvLowerBound[1].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Vl", hsvLowerBound[2].getData(), 0.0f, 255.0f);

            ImGui.text("HSV Upper Bound");
            ImGui.sliderFloat("Hu", hsvUpperBound[0].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Su", hsvUpperBound[1].getData(), 0.0f, 255.0f);
            ImGui.sliderFloat("Vu", hsvUpperBound[2].getData(), 0.0f, 255.0f);
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

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
      ballDetector.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXBallTrackingDemo();
   }
}
