package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2BoundingBoxVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXCenterPoseBBoxDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Zed 2 Display Demo");
   ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "zed_2_demo_node");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
   private final RDXGlobalVisualizersPanel globalVisualizersPanel = new RDXGlobalVisualizersPanel();

   public RDXCenterPoseBBoxDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            RDXROS2BoundingBoxVisualizer CenterPoseBoundingBoxVisualizer = new RDXROS2BoundingBoxVisualizer("CenterPose Bounding Box",
                                                                                                            ros2Helper,
                                                                                                            ReferenceFrame.getWorldFrame(),
                                                                                                            PerceptionAPI.CENTERPOSE_DETECTED_OBJECT);
            CenterPoseBoundingBoxVisualizer.setActive(true);
            globalVisualizersPanel.addVisualizer(CenterPoseBoundingBoxVisualizer);

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
      new RDXCenterPoseBBoxDemo();
   }
}