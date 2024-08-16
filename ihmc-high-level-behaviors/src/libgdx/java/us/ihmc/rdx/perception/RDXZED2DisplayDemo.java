package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXGlobalVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

public class RDXZED2DisplayDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ZED 2 Display Demo");
   private final RDXGlobalVisualizersPanel globalVisualizersPanel = new RDXGlobalVisualizersPanel();

   public RDXZED2DisplayDemo()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "zed_2_demo_node");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2ImageMessageVisualizer zed2LeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                           PubSubImplementation.FAST_RTPS,
                                                                                                           PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2LeftColorImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2LeftColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2RightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                            PubSubImplementation.FAST_RTPS,
                                                                                                            PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            zed2RightColorImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2RightColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth",
                                                                                                       PubSubImplementation.FAST_RTPS,
                                                                                                       PerceptionAPI.ZED2_DEPTH);
            zed2DepthImageVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2DepthImageVisualizer);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                        PubSubImplementation.FAST_RTPS,
                                                                                                                        PerceptionAPI.ZED2_DEPTH,
                                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                              RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(globalVisualizersPanel);
            globalVisualizersPanel.create();
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
      new RDXZED2DisplayDemo();
   }
}