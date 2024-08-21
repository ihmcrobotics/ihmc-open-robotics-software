package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.perception.detections.centerPose.CenterPoseDetectionSubscriber;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2DetectedObjectBoundingBoxVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDModelData;

public class RDXCenterposeObjectDetectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Centerpose Object Detection Demo");
   private final RDXPerceptionVisualizersPanel perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();
   private final ROS2Helper ros2Helper;

   public RDXCenterposeObjectDetectionDemo()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "zed_2_demo_node");
      ros2Helper = new ROS2Helper(ros2Node);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RDXROS2ImageMessageVisualizer zed2LeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                           PubSubImplementation.FAST_RTPS,
                                                                                                           PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2LeftColorImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2LeftColorImageVisualizer);

            ReferenceFrame zedLeftCameraFrame = ZEDModelData.createCameraReferenceFrame(RobotSide.LEFT, ReferenceFrame.getWorldFrame());
            ReferenceFrame centerposeOutputFrame
                  = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("CenterposeOutputFrame",
                                                                                      zedLeftCameraFrame,
                                                                                      CenterPoseDetectionSubscriber.CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM);


            RDXROS2DetectedObjectBoundingBoxVisualizer centerPoseBoundingBoxVisualizer
                  = new RDXROS2DetectedObjectBoundingBoxVisualizer("CenterPose Bounding Box",
                                                                   ros2Helper,
                                                                   centerposeOutputFrame,
                                                                   PerceptionAPI.CENTERPOSE_DETECTED_OBJECT,
                                                                   baseUI.getPrimary3DPanel().getCamera3D());
            centerPoseBoundingBoxVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(centerPoseBoundingBoxVisualizer);
            zed2LeftColorImageVisualizer.getOpenCVVideoVisualizer().addOverlay(centerPoseBoundingBoxVisualizer::drawVertexOverlay);

            RDXROS2ImageMessageVisualizer zed2RightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                            PubSubImplementation.FAST_RTPS,
                                                                                                            PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            zed2RightColorImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2RightColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth",
                                                                                                       PubSubImplementation.FAST_RTPS,
                                                                                                       PerceptionAPI.ZED2_DEPTH);
            zed2DepthImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2DepthImageVisualizer);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                        PubSubImplementation.FAST_RTPS,
                                                                                                                        PerceptionAPI.ZED2_DEPTH,
                                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                              RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
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
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXCenterposeObjectDetectionDemo();
   }
}