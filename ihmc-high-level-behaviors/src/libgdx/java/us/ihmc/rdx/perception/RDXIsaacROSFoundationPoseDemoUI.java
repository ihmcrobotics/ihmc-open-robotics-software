package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import ihmc_common_msgs.msg.dds.Box3DMessage;
import imgui.ImGui;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseObject;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;

import static us.ihmc.perception.demo.IsaacROSFoundationPoseDemo.RESULT_TOPIC;

public class RDXIsaacROSFoundationPoseDemoUI
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
   private final ROS2Publisher<Empty> trackingResetPublisher = ros2Node.createPublisher(IsaacROSFoundationPoseObject.MUSTARD.topics.reset());

   private final Box3D result = new Box3D();
   private RDXReferenceFrameGraphic referenceFrameGraphic;

   public RDXIsaacROSFoundationPoseDemoUI()
   {
      result.setToNaN();
      ros2Node.createSubscription2(RESULT_TOPIC, this::receiveOutput);

      RDXBaseUI baseUI = new RDXBaseUI();
      RDXPerceptionVisualizersPanel visualizers = new RDXPerceptionVisualizersPanel();
      RDXBoxVisualizer boxVisualizer = new RDXBoxVisualizer();
      boxVisualizer.setColor(Color.RED);
      boxVisualizer.setLineWidth(0.01);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            visualizers.addVisualizer(new RDXROS2ColoredPointCloudVisualizer("ZED Point Cloud",
                                                                                            ros2Node,
                                                                                            PerceptionAPI.ZED_DEPTH,
                                                                                            PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED Color", ros2Node, PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.LEFT)));
            visualizers.addVisualizer(new RDXROS2ImageMessageVisualizer("ZED Depth", ros2Node, PerceptionAPI.ZED_DEPTH));
            visualizers.addVisualizer(new RDXROS2YOLOv8Visualizer("YOLO", ros2Node, peerClockOffsetEstimator, PerceptionAPI.YOLO_ANNOTATED_IMAGE));
            visualizers.create(baseUI);

            referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);
            baseUI.getPrimaryScene().addRenderableProvider(boxVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic);

            baseUI.getImGuiPanelManager().addPanel("Setting", this::renderSettings);

            baseUI.create();
         }

         @Override
         public void render()
         {
            visualizers.update();
            updateBoundingBoxes();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            if (ImGui.button("Reset Tracking"))
            {
               trackingResetPublisher.publish(new Empty());
            }
         }

         private void updateBoundingBoxes()
         {
            synchronized (result)
            {
               boxVisualizer.generateMesh(result);
               boxVisualizer.update();

               referenceFrameGraphic.getFramePose3D().set(result.getPose());
               referenceFrameGraphic.updateFromFramePose();
            }
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

   private void receiveOutput(Box3DMessage output)
   {
      synchronized (result)
      {
         result.getPose().set(output.getPose());
         result.getSize().set(output.getSize());
      }
   }

   public static void main(String[] args)
   {
      new RDXIsaacROSFoundationPoseDemoUI();
   }
}
