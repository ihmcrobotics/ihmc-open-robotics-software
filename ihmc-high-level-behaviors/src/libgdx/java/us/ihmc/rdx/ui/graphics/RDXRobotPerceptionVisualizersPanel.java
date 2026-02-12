package us.ihmc.rdx.ui.graphics;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2HeightMapVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2KSTRobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.foundationPose.RDXIsaacROSFoundationPoseVisualizer;
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
   protected RDXROS2ColoredPointCloudVisualizer experimentalCameraColoredPointCloudVisualizer;
   protected RDXROS2ImageMessageVisualizer experimentalCameraLeftColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer experimentalCameraRightColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer experimentalCameraDepthImageVisualizer;
   protected RDXROS2ColoredPointCloudVisualizer steppingCameraColoredPointCloudVisualizer;
   protected RDXROS2ImageMessageVisualizer steppingCameraLeftColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer steppingCameraRightColorImageVisualizer;
   protected RDXROS2ImageMessageVisualizer steppingCameraDepthImageVisualizer;
   protected RDXROS2YOLOv8Visualizer yoloVisualizer;
   protected RDXIsaacROSFoundationPoseVisualizer foundationPoseVisualizer;
   protected RDXROS2HeightMapVisualizer heightMapVisualizer;
   protected RDXROS2HeightMapVisualizer yoloMapVisualizer;
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

   public RDXROS2ColoredPointCloudVisualizer getExperimentalCameraColoredPointCloudVisualizer()
   {
      return experimentalCameraColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getExperimentalCameraLeftColorImageVisualizer()
   {
      return experimentalCameraLeftColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getExperimentalCameraRightColorImageVisualizer()
   {
      return experimentalCameraRightColorImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getExperimentalCameraDepthImageVisualizer()
   {
      return experimentalCameraDepthImageVisualizer;
   }

   public RDXROS2ColoredPointCloudVisualizer getSteppingCameraColoredPointCloudVisualizer()
   {
      return steppingCameraColoredPointCloudVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getSteppingCameraDepthImageVisualizer()
   {
      return steppingCameraDepthImageVisualizer;
   }

   public RDXROS2ImageMessageVisualizer getSteppingCameraLeftColorImageVisualizer()
   {
      return steppingCameraLeftColorImageVisualizer;
   }

   public RDXROS2YOLOv8Visualizer getYoloVisualizer()
   {
      return yoloVisualizer;
   }

   public RDXIsaacROSFoundationPoseVisualizer getFoundationPoseVisualizer()
   {
      return foundationPoseVisualizer;
   }

   public RDXROS2HeightMapVisualizer getHeightMapVisualizer()
   {
      return heightMapVisualizer;
   }

   public RDXROS2HeightMapVisualizer getYOLOObstacleMapVisualizer()
   {
      return yoloMapVisualizer;
   }

   public RDXROS2FramePlanarRegionsVisualizer getPlanarRegionsVisualizer()
   {
      return planarRegionsVisualizer;
   }
}
