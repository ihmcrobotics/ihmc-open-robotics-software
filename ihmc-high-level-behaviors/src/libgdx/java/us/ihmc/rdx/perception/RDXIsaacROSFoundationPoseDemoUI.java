package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.communication.ros2.tf2.ROS2Frame;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence;
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
import vision_msgs.msg.dds.Detection3D;
import vision_msgs.msg.dds.Detection3DArray;

import java.util.HashSet;
import java.util.Set;

import static us.ihmc.perception.demo.IsaacROSFoundationPoseDemo.*;

public class RDXIsaacROSFoundationPoseDemoUI
{
   private static final RotationMatrix FOUNDATIONPOSE_TO_IHMC_ROTATION = new RotationMatrix(new double[] {0, 0, 1,
                                                                                                         -1, 0, 0,
                                                                                                          0,-1, 0});

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
   private final ROS2Publisher<Empty> trackingResetPublisher = ros2Node.createPublisher(TRACKING_RESET_TOPIC);
   private final Set<Box3D> detectedBoundingBoxes = new HashSet<>();

   private final ROS2Frame zedFrame = new ROS2MutableFrame(ros2Node, "zed_frame", ReferenceFrame.getWorldFrame());

   public RDXIsaacROSFoundationPoseDemoUI()
   {
      ros2Node.createSubscription2(TRACKING_RESULT_TOPIC, this::receiveOutput);
      ros2Node.createSubscription2(REGISTRATION_RESULT_TOPIC, this::receiveOutput);

      RDXBaseUI baseUI = new RDXBaseUI();
      RDXPerceptionVisualizersPanel visualizers = new RDXPerceptionVisualizersPanel();
      Set<RDXBoxVisualizer> boxVisualizers = new HashSet<>();
      Set<RDXReferenceFrameGraphic> referenceFrameGraphics = new HashSet<>();

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

            baseUI.create();
         }

         @Override
         public void render()
         {
            visualizers.update();
            updateBoundingBoxes();

            baseUI.renderBeforeOnScreenUI();
            renderSettings();
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
            for (RDXBoxVisualizer boxVisualizer : boxVisualizers)
            {
               baseUI.getPrimaryScene().removeRenderable(boxVisualizer);
               boxVisualizer.dispose();
            }
            boxVisualizers.clear();

            for (RDXReferenceFrameGraphic referenceFrameGraphic : referenceFrameGraphics)
            {
               baseUI.getPrimaryScene().removeRenderable(referenceFrameGraphic);
               referenceFrameGraphic.dispose();
            }
            referenceFrameGraphics.clear();

            synchronized (detectedBoundingBoxes)
            {
               for (Box3D boundingBox : detectedBoundingBoxes)
               {
                  RDXBoxVisualizer boxVisualizer = new RDXBoxVisualizer();
                  boxVisualizer.setLineWidth(0.01);
                  boxVisualizer.setColor(new Color(1.0f, 0.0f, 0.0f, 1.0f));
                  boxVisualizer.generateMesh(boundingBox);
                  boxVisualizer.update();
                  boxVisualizers.add(boxVisualizer);
                  baseUI.getPrimaryScene().addRenderableProvider(boxVisualizer, boxVisualizer);

                  RDXReferenceFrameGraphic referenceFrameGraphic = new RDXReferenceFrameGraphic(0.1);
                  referenceFrameGraphic.getFramePose3D().set(boundingBox.getPose());
                  referenceFrameGraphic.updateFromFramePose();
                  referenceFrameGraphics.add(referenceFrameGraphic);
                  baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic, referenceFrameGraphic);
               }
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

   private void receiveOutput(Detection3DArray output)
   {
      IDLSequence.Object<Detection3D> detections = output.getDetections();

      zedFrame.update();

      synchronized (detectedBoundingBoxes)
      {
         detectedBoundingBoxes.clear();

         for (int i = 0; i < detections.size(); ++i)
         {
            Detection3D detection = detections.get(i);
            Vector3D size = detection.getBbox().getSize();
            FramePose3D pose = new FramePose3D(zedFrame, detection.getBbox().getCenter());
            pose.prependRotation(FOUNDATIONPOSE_TO_IHMC_ROTATION);
            pose.changeFrame(ReferenceFrame.getWorldFrame());
            detectedBoundingBoxes.add(new Box3D(pose, size));
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXIsaacROSFoundationPoseDemoUI();
   }
}
