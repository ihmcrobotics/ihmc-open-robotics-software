package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2SRTVideoStreamVisualizer;
import us.ihmc.ros2.ROS2Node;

/**
 * Demo for the SRT visualizer. Connects to a ZED stream and visualizes it.
 * To stream video from a ZED camera or SVO file, use the {@link us.ihmc.perception.streaming.ROS2SRTVideoStreamingDemo}.
 */
public class RDXROS2SRTVisualizerDemo
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_visualize_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizersPanel visualizersPanel = new RDXPerceptionVisualizersPanel();

   private RDXROS2SRTVisualizerDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2SRTVideoStreamVisualizer zedLeftColorVisualizer = new RDXROS2SRTVideoStreamVisualizer(ros2Helper,
                                                                                                         "ZED Color Left",
                                                                                                         PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS);
            RDXROS2SRTVideoStreamVisualizer zedRightColorVisualizer = new RDXROS2SRTVideoStreamVisualizer(ros2Helper,
                                                                                                          "ZED Color Right",
                                                                                                          PerceptionAPI.SRT_ZED_RIGHT_COLOR_STREAM_STATUS);
            RDXROS2SRTVideoStreamVisualizer zedDepthVisualizer = new RDXROS2SRTVideoStreamVisualizer(ros2Helper,
                                                                                                     "ZED Depth",
                                                                                                     PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS);
            visualizersPanel.addVisualizer(zedLeftColorVisualizer);
            visualizersPanel.addVisualizer(zedRightColorVisualizer);
            visualizersPanel.addVisualizer(zedDepthVisualizer);
            visualizersPanel.create();

            baseUI.getImGuiPanelManager().addPanel(visualizersPanel);
            baseUI.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            visualizersPanel.update();

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            visualizersPanel.destroy();
            baseUI.dispose();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS2SRTVisualizerDemo();
   }
}