package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

public class RDXZED2DisplayDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ZED 2 Display Demo");
   private final RDXPerceptionVisualizersPanel perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();

   public RDXZED2DisplayDemo()
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("zed_2_demo_node");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2ImageMessageVisualizer zed2LeftColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Left",
                                                                                                           ros2Node,
                                                                                                           PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zed2LeftColorImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2LeftColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2RightColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Color Right",
                                                                                                            ros2Node,
                                                                                                            PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            zed2RightColorImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2RightColorImageVisualizer);

            RDXROS2ImageMessageVisualizer zed2DepthImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED 2 Depth",
                                                                                                       ros2Node,
                                                                                                       PerceptionAPI.ZED2_DEPTH);
            zed2DepthImageVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2DepthImageVisualizer);

            RDXROS2ColoredPointCloudVisualizer zed2ColoredPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                        ros2Node,
                                                                                                                        ros2Node,
                                                                                                                        PerceptionAPI.ZED2_DEPTH,
                                                                                                                        PerceptionAPI.ZED2_COLOR_IMAGES.get(
                                                                                                                              RobotSide.LEFT));
            zed2ColoredPointCloudVisualizer.setActive(true);
            perceptionVisualizerPanel.addVisualizer(zed2ColoredPointCloudVisualizer);

            baseUI.create();
            perceptionVisualizerPanel.create(baseUI);
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
      new RDXZED2DisplayDemo();
   }
}