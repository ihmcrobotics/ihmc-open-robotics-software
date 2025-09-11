package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import vision_msgs.msg.dds.Detection3D;
import vision_msgs.msg.dds.Detection3DArray;

import java.util.HashSet;
import java.util.Set;

public class RDXIsaacROSFoundationPoseDemoUI
{
   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());
   private final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
   private final ROS2Subscription<Detection3DArray> outputSubscription = ros2Node.createSubscription2(new ROS2Topic<>().withModule("tracking/output")
                                                                                                                       .withType(Detection3DArray.class),
                                                                                                      this::receiveOutput);
   private final Set<Box3D> detectedBoundingBoxes = new HashSet<>();

   public RDXIsaacROSFoundationPoseDemoUI()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      RDXPerceptionVisualizersPanel visualizers = new RDXPerceptionVisualizersPanel();
      Set<RDXBoxVisualizer> boxVisualizers = new HashSet<>();

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
            baseUI.renderEnd();
         }

         private void updateBoundingBoxes()
         {
            for (RDXBoxVisualizer boxVisualizer : boxVisualizers)
            {
               baseUI.getPrimaryScene().removeRenderable(boxVisualizer);
               boxVisualizer.dispose();
            }

            synchronized (detectedBoundingBoxes)
            {
               for (Box3D boundingBox : detectedBoundingBoxes)
               {
                  RDXBoxVisualizer visualizer = new RDXBoxVisualizer();
                  visualizer.generateMesh(boundingBox);
                  boxVisualizers.add(visualizer);
                  baseUI.getPrimaryScene().addRenderableProvider(visualizer, visualizer);
               }
            }
         }

         @Override
         public void dispose()
         {
            visualizers.destroy();
            baseUI.dispose();

            peerClockOffsetEstimator.destroy();
            outputSubscription.remove();
            ros2Node.destroy();
         }
      });
   }

   private void receiveOutput(Detection3DArray output)
   {
      IDLSequence.Object<Detection3D> detections = output.getDetections();

      synchronized (detectedBoundingBoxes)
      {
         detectedBoundingBoxes.clear();

         for (int i = 0; i < detections.size(); ++i)
         {
            Detection3D detection = detections.get(i);
            Pose3D pose = detection.getBbox().getCenter();
            Vector3D size = detection.getBbox().getSize();
            detectedBoundingBoxes.add(new Box3D(pose, size));
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXIsaacROSFoundationPoseDemoUI();
   }
}
