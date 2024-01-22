package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeDetectionManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXGeneralToolsPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2DetectedObjectBoundingBoxVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDModelData;

public class RDXCenterposeObjectDetectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Centerpose Object Detection Demo");
   private final RDXGeneralToolsPanel globalVisualizersPanel;
   private final ROS2Helper ros2Helper;

   public RDXCenterposeObjectDetectionDemo()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "zed_2_demo_node");
      ros2Helper = new ROS2Helper(ros2Node);

       globalVisualizersPanel = new RDXGeneralToolsPanel(baseUI);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RDXROS2ImageMessageVisualizer zed2LeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                           PubSubImplementation.FAST_RTPS,
                                                                                                           PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2LeftColorImageVisualizer.setSubscribed(true);
            zed2LeftColorImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2LeftColorImageVisualizer);

            ReferenceFrame zedLeftCameraFrame = ZEDModelData.createCameraReferenceFrame(RobotSide.LEFT, ReferenceFrame.getWorldFrame());
            ReferenceFrame centerposeOutputFrame
                  = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("CenterposeOutputFrame",
                                                                                      zedLeftCameraFrame,
                                                                                      CenterposeDetectionManager.CENTERPOSE_DETECTION_TO_IHMC_ZUP_TRANSFORM);


            RDXROS2DetectedObjectBoundingBoxVisualizer centerPoseBoundingBoxVisualizer
                  = new RDXROS2DetectedObjectBoundingBoxVisualizer("CenterPose Bounding Box",
                                                                   ros2Helper,
                                                                   centerposeOutputFrame,
                                                                   PerceptionAPI.CENTERPOSE_DETECTED_OBJECT,
                                                                   baseUI.getPrimary3DPanel().getCamera3D());
            centerPoseBoundingBoxVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(centerPoseBoundingBoxVisualizer);
            zed2LeftColorImageVisualizer.addOverlay(centerPoseBoundingBoxVisualizer::drawVertexOverlay);

            RDXROS2ImageMessageVisualizer zed2RightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                            PubSubImplementation.FAST_RTPS,
                                                                                                            PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            zed2RightColorImageVisualizer.setSubscribed(true);
            zed2RightColorImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2RightColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth",
                                                                                                       PubSubImplementation.FAST_RTPS,
                                                                                                       PerceptionAPI.ZED2_DEPTH);
            zed2DepthImageVisualizer.setSubscribed(true);
            zed2DepthImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2DepthImageVisualizer);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                        PubSubImplementation.FAST_RTPS,
                                                                                                                        PerceptionAPI.ZED2_DEPTH,
                                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                              RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.setSubscribed(true);
            zed2ColoredPointCloudVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            globalVisualizersPanel.create();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersPanel);
         }

         @Override
         public void render()
         {
            globalVisualizersPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            globalVisualizersPanel.destroy();
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