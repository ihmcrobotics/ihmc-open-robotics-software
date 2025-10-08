package us.ihmc.rdx.ui.graphics;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXDetectionManagerSettings;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2KSTRobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.zed.ZEDModelData;

public abstract class RDXRobotPerceptionVisualizersPanel extends RDXPerceptionVisualizersPanel
{
   protected final ROS2SyncedRobotModel syncedRobot;
   protected final ROS2Helper ros2Helper;
   protected final ROS2PeerClockOffsetEstimator peerClockOffsetEstimator;

   // Common — always present
   protected final RDXROS2RobotVisualizer robotVisualizer;
   protected final RDXROS2KSTRobotVisualizer kinematicsStreamingSolutionVisualizer;

   // Optional — may be null in some robots
   protected RDXROS2ColoredPointCloudVisualizer zedColoredPointCloudVisualizer;
   protected RDXROS2ImageMessageVisualizer zedLeftColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer zedRightColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer zedDepthImageVisualizer;
   protected RDXROS2ColoredPointCloudVisualizer realsenseColoredPointCloudVisualizer;
   protected RDXROS2ImageMessageVisualizer realsenseDepthImageVisualizer;
   protected RDXROS2ImageMessageVisualizer realsenseColorImageVisualizer;
   protected RDXROS2YOLOv8Visualizer yoloVisualizer;
   protected RDXROS2HeightMapVisualizer heightMapVisualizer;
   protected RDXDetectionManagerSettings detectionManagerSettings;
   protected RDXROS2FramePlanarRegionsVisualizer planarRegionsVisualizer;

   protected RDXRobotPerceptionVisualizersPanel(ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot, ROS2PeerClockOffsetEstimator peerClockOffsetEstimator)
   {
      this.syncedRobot = syncedRobot;
      this.peerClockOffsetEstimator = peerClockOffsetEstimator;
      this.ros2Helper = new ROS2Helper(ros2Node);

      // Common robot visualizer
      robotVisualizer = new RDXROS2RobotVisualizer(ros2Helper, syncedRobot);
      robotVisualizer.setPinned(true);
      robotVisualizer.setActive(true);
      addVisualizer(robotVisualizer);

      // Kinematics streaming solution visualizer
      kinematicsStreamingSolutionVisualizer = new RDXROS2KSTRobotVisualizer(ros2Helper.getROS2Node(), syncedRobot.getRobotModel());
      addVisualizer(kinematicsStreamingSolutionVisualizer);

      // Additional visualizers instantiated in robot specific class
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {
      robotVisualizer.setupCameraTracking(baseUI.getPrimary3DPanel().getCamera3D());
      // Let subclass setup optional interactable sensors, etc.
      setupAdditionalSensors(baseUI);

      // Call common setup
      super.create(baseUI);
   }

   /**
    * Subclasses override this to set up interactable sensors or config
    */
   protected void setupAdditionalSensors(RDXBaseUI baseUI)
   {
   }

   public ZEDModelData getZEDModelData()
   {
      return ZEDModelData.ZED;
   }

   @Override
   public void destroy()
   {
      super.destroy();
   }

   // Getters for common/optional visualizers
   public RDXROS2RobotVisualizer getRobotVisualizer()
   {
      return robotVisualizer;
   }

   public RDXROS2KSTRobotVisualizer getKinematicsStreamingSolutionVisualizer()
   {
      return kinematicsStreamingSolutionVisualizer;
   }

   public RDXROS2ColoredPointCloudVisualizer getZedColoredPointCloudVisualizer()
   {
      return zedColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZedLeftColorImageVisualizer()
   {
      return zedLeftColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZedRightColorImageVisualizer()
   {
      return zedRightColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getZedDepthImageVisualizer()
   {
      return zedDepthImageVisualizer;
   }

   public RDXROS2ColoredPointCloudVisualizer getRealsenseColoredPointCloudVisualizer()
   {
      return realsenseColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getRealsenseDepthImageVisualizer()
   {
      return realsenseDepthImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getRealsenseColorImageVisualizer()
   {
      return realsenseColorImageVisualizer;
   }

   public RDXROS2YOLOv8Visualizer getYoloVisualizer()
   {
      return yoloVisualizer;
   }

   public RDXROS2HeightMapVisualizer getHeightMapVisualizer()
   {
      return heightMapVisualizer;
   }

   public RDXROS2FramePlanarRegionsVisualizer getPlanarRegionsVisualizer()
   {
      return planarRegionsVisualizer;
   }
}
