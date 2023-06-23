package us.ihmc.avatar.colorVision;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sceneGraph.PredefinedSceneNodeLibrary;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

/**
 * To run this you have to download the Spinnaker SDK, move it to the robot computer, then run
 * the install script inside of it.
 */
public class DualBlackflyAndAruCoMarkerOnRobotProcess
{
   private static final String LEFT_SERIAL_NUMBER = System.getProperty("blackfly.left.serial.number", "00000000");
   private static final String RIGHT_SERIAL_NUMBER = System.getProperty("blackfly.right.serial.number", "22206798");

   private final SpinnakerBlackflyManager spinnakerBlackflyManager = new SpinnakerBlackflyManager();
   private final SideDependentList<DualBlackflyCamera> dualBlackflyCameras = new SideDependentList<>();
   private final ROS2Node ros2Node;

   public DualBlackflyAndAruCoMarkerOnRobotProcess(DRCRobotModel robotModel,
                                                   PredefinedSceneNodeLibrary predefinedSceneNodeLibrary,
                                                   IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics)
   {
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "blackfly_node");

      ROS2SyncedRobotModel syncedRobotModel = new ROS2SyncedRobotModel(robotModel, ros2Node);
      // Helpful to view relative sensor transforms when robot controller is not running
      syncedRobotModel.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      if (!LEFT_SERIAL_NUMBER.equals("00000000"))
      {
         throw new RuntimeException("Left Blackfly not supported");
      }

      if (!RIGHT_SERIAL_NUMBER.equals("00000000"))
      {
         LogTools.info("Adding Blackfly right with serial number: {}", RIGHT_SERIAL_NUMBER);
         SpinnakerBlackfly spinnakerBlackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(RIGHT_SERIAL_NUMBER);
         dualBlackflyCameras.put(RobotSide.RIGHT,
                                 new DualBlackflyCamera(RobotSide.RIGHT,
                                                        syncedRobotModel,
                                                        robotModel.getSensorInformation().getObjectDetectionCameraTransform(),
                                                        ros2Node,
                                                        spinnakerBlackfly,
                                                        ousterFisheyeColoringIntrinsics,
                                                        predefinedSceneNodeLibrary));
      }
      else
      {
         LogTools.warn("No serial number for right Blackfly specified. The sensor will not be available.");
      }

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));
   }

   private void destroy()
   {
      for (DualBlackflyCamera dualBlackflyCamera : dualBlackflyCameras)
      {
         if (dualBlackflyCamera != null)
            dualBlackflyCamera.destroy();
      }
      ros2Node.destroy();
      spinnakerBlackflyManager.destroy();
   }
}
