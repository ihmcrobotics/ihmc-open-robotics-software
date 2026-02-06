package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.foundationPose.RDXIsaacROSFoundationPoseVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

public class RDXIsaacROSFoundationPoseDemoUI
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

   public RDXIsaacROSFoundationPoseDemoUI()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      RDXPerceptionVisualizersPanel visualizers = new RDXPerceptionVisualizersPanel();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            visualizers.addVisualizer(new RDXROS2ColoredPointCloudVisualizer("ZED Point Cloud",
                                                                             ros2Node,
                                                                             PerceptionAPI.EXPERIMENTAL_ZED_DEPTH,
                                                                             PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED Color", ros2Node, PerceptionAPI.EXPERIMENTAL_ZED_COLOR.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED Depth", ros2Node, PerceptionAPI.EXPERIMENTAL_ZED_DEPTH));
            visualizers.addVisualizer(new RDXROS2YOLOv8Visualizer("YOLO", ros2Node, peerClockOffsetEstimator, PerceptionAPI.YOLO_ANNOTATED_IMAGE));
            visualizers.addVisualizer(new RDXIsaacROSFoundationPoseVisualizer("FoundationPose", ros2Node, peerClockOffsetEstimator));
            visualizers.create(baseUI);

            baseUI.getPrimaryScene().addRenderableProvider(visualizers);
            baseUI.create();
         }

         @Override
         public void render()
         {
            visualizers.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            visualizers.destroy();
            baseUI.dispose();

            peerClockOffsetEstimator.destroy();
            ros2Node.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXIsaacROSFoundationPoseDemoUI();
   }
}
