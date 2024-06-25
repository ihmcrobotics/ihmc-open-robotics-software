package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;

public class RDXZEDSVORecordingDemo
{
   private static final RecordMode RECORD_MODE = RecordMode.PLAYBACK; // If you set to PLAYBACK, you must specify a valid SVO_FILE_NAME below
   private static final String SVO_FILE_NAME = "/home/d/.ihmc/logs/perception/20240625_154000_ZEDRecording.svo2"; // If it's null, a unique file name with a timestamp will be generated

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ROS2Node ros2Node;
   private RDXPerceptionVisualizersPanel perceptionVisualizerPanel;

   private final ZEDColorDepthImageRetrieverSVO zedColorDepthImageRetrieverSVO;
   private final ZEDColorDepthImagePublisher zedColorDepthImagePublisher;

   public RDXZEDSVORecordingDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::dispose));

      zedColorDepthImageRetrieverSVO = new ZEDColorDepthImageRetrieverSVO(0, ReferenceFrame::getWorldFrame, null, null, RECORD_MODE, SVO_FILE_NAME);
      zedColorDepthImageRetrieverSVO.start();

      zedColorDepthImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES,
                                                                    PerceptionAPI.ZED2_DEPTH,
                                                                    PerceptionAPI.ZED2_CUT_OUT_DEPTH);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.VIRTUAL, RDXSceneLevel.MODEL, RDXSceneLevel.GROUND_TRUTH);

            PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
            ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "zed_svo_recording_demo");

            perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);

            addZEDVisualizers(perceptionVisualizerPanel);

            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            perceptionVisualizerPanel.update();

            // TODO: move to thread?
            RawImage depthImage = zedColorDepthImageRetrieverSVO.getLatestRawDepthImage();
            RawImage leftColorImage = zedColorDepthImageRetrieverSVO.getLatestRawColorImage(RobotSide.LEFT);
            RawImage rightColorImage = zedColorDepthImageRetrieverSVO.getLatestRawColorImage(RobotSide.RIGHT);
            zedColorDepthImagePublisher.setNextColorImage(leftColorImage.get(), RobotSide.LEFT);
            zedColorDepthImagePublisher.setNextColorImage(rightColorImage.get(), RobotSide.RIGHT);
            zedColorDepthImagePublisher.setNextGpuDepthImage(depthImage.get());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            // This never gets called?
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public void dispose()
   {
      zedColorDepthImagePublisher.destroy();
      zedColorDepthImageRetrieverSVO.destroy();
   }

   private void addZEDVisualizers(RDXPerceptionVisualizersPanel perceptionVisualizerPanel)
   {
      // ZED2 colored point cloud visualizer
      {
         RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                     PubSubImplementation.FAST_RTPS,
                                                                                                                     PerceptionAPI.ZED2_DEPTH,
                                                                                                                     PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                           RobotSide.LEFT))
         {
            private final ImBoolean removeOverlap = new ImBoolean(false);
            private final ROS2Heartbeat overlapRemovalHeartbeat = new ROS2Heartbeat(ros2Node, PerceptionAPI.REQUEST_OVERLAP_REMOVAL);

            @Override
            public void renderImGuiWidgets()
            {
               super.renderImGuiWidgets();

               if (ImGui.checkbox("Remove Overlap", removeOverlap))
               {
                  overlapRemovalHeartbeat.setAlive(removeOverlap.get());
                  changeTopics(PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT),
                               removeOverlap.get() ? PerceptionAPI.ZED2_CUT_OUT_DEPTH : PerceptionAPI.ZED2_DEPTH);
               }
            }
         };
         zed2ColoredPointCloudVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
         zed2ColoredPointCloudVisualizer.setActive(true);
         perceptionVisualizerPanel.addVisualizer(zed2ColoredPointCloudVisualizer);
      }

      // ZED left color visualizer
      {
         RDXROS2ImageMessageVisualizer zedLeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                       PubSubImplementation.FAST_RTPS,
                                                                                                       PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
         zedLeftColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_COLOR);
         perceptionVisualizerPanel.addVisualizer(zedLeftColorImageVisualizer);
      }

      // ZED 2 color right image visualizer
      {
         RDXROS2ImageMessageVisualizer zedRightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                        PubSubImplementation.FAST_RTPS,
                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
         zedRightColorImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_COLOR);
         perceptionVisualizerPanel.addVisualizer(zedRightColorImageVisualizer);
      }

      // ZED 2 depth image visualizer
      {
         RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth Image",
                                                                                                    PubSubImplementation.FAST_RTPS,
                                                                                                    PerceptionAPI.ZED2_DEPTH);
         zed2DepthImageVisualizer.createRequestHeartbeat(ros2Node, PerceptionAPI.REQUEST_ZED_DEPTH);
         perceptionVisualizerPanel.addVisualizer(zed2DepthImageVisualizer);
      }
   }

   public static void main(String[] args)
   {
      new RDXZEDSVORecordingDemo();
   }
}
