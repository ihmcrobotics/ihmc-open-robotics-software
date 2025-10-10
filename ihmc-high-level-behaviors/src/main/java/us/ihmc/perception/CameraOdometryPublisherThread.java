package us.ihmc.perception;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import ihmc_common_msgs.msg.dds.StampedOdometryPacket;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.OdometrySensor;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;

public class CameraOdometryPublisherThread extends RepeatingTaskThread
{
   private final OdometrySensor sensor;
   private final ROS2Publisher<StampedOdometryPacket> odometryInfoPublisher; // used for debugging
   private final ROS2Publisher<StampedPosePacket> odometryCorrectionPublisher; // used for state estimation correction
   private final ROS2SyncedRobotModel syncedRobot;
   private boolean computeOdometry = false;
   private final String parentCameraName;
   private final RigidBodyTransform cameraInParent;
   private final RigidBodyTransform initialCameraWorldOffset = new RigidBodyTransform();
   
   public CameraOdometryPublisherThread(ROS2Node ros2Node, OdometrySensor sensor, ROS2SyncedRobotModel syncedRobot)
   {
      super(sensor.getSensorName() + "OdometryPublishThread");
      String robotName = syncedRobot.getRobotModel().getSimpleRobotName();

      this.sensor = sensor;
      this.syncedRobot = syncedRobot;
      this.odometryInfoPublisher = ros2Node.createPublisher(StateEstimatorAPI.getTopic(StampedOdometryPacket.class, robotName.toLowerCase()));
      this.odometryCorrectionPublisher = ros2Node.createPublisher(StateEstimatorAPI.getTopic(StampedPosePacket.class, robotName.toLowerCase()));
      ros2Node.createSubscription2(getTopic(HighLevelStateChangeStatusMessage.class, robotName), this::acceptHighLevelStateChangeStatusMessage);

      parentCameraName = syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraParentFrameName();
      cameraInParent = syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform();
   }

   private void acceptHighLevelStateChangeStatusMessage(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName initialState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName endState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      if (endState != initialState && endState != null)
      {
         boolean newComputeOdometry = switch (endState)
         {
            case STAND_PREP_STATE, STAND_TRANSITION_STATE, WALKING -> true;
            default -> false;
         };

         if (newComputeOdometry && !computeOdometry)
         {
            // Register offset World frame robot-sensor
            // Step 1: Camera transform in world from robot
            RigidBodyTransform parentInWorld = parentCameraName.equals("head")
                  ? syncedRobot.getFullRobotModel().getHead().getParentJoint().getFrameAfterJoint().getTransformToRoot()
                  : syncedRobot.getFullRobotModel().getChest().getParentJoint().getFrameAfterJoint().getTransformToRoot();
            RigidBodyTransform cameraInWorldFromRobot = new RigidBodyTransform(parentInWorld);
            cameraInWorldFromRobot.multiply(cameraInParent);

            // Step 3: Camera transform in world from sensor
            RigidBodyTransform cameraInWorldFromSensor = sensor.getTrackedSensorFrame().getTransformToRoot();

            // Step 4: Register initial offset: cameraInWorldFromRobot.inverse() * cameraInWorldFromSensor
            RigidBodyTransform cameraInWorldFromRobotInv = new RigidBodyTransform(cameraInWorldFromRobot);
            cameraInWorldFromRobotInv.invert();
            initialCameraWorldOffset.set(cameraInWorldFromRobotInv);
            initialCameraWorldOffset.multiply(cameraInWorldFromSensor);
         }
         computeOdometry = newComputeOdometry;
      }
   }

   @Override
   protected void runTask()
   {
      if (computeOdometry)
      {
         // 1. Camera-to-parent (must invert parent-to-camera)
         RigidBodyTransform parentInCamera = new RigidBodyTransform(cameraInParent);
         parentInCamera.invert(); // Now it's parent frame in camera frame

         // 2. Compute pelvis-in-parent
         RigidBodyTransform parentInWorld = parentCameraName.equals("head")
               ? syncedRobot.getFullRobotModel().getHead().getParentJoint().getFrameAfterJoint().getTransformToRoot()
               : syncedRobot.getFullRobotModel().getChest().getParentJoint().getFrameAfterJoint().getTransformToRoot();
         RigidBodyTransform pelvisInWorld = syncedRobot.getFullRobotModel().getPelvis().getParentJoint().getFrameAfterJoint().getTransformToRoot();
         // Invert parent-in-world, multiply by pelvis-in-world
         RigidBodyTransform parentInWorldInv = new RigidBodyTransform(parentInWorld);
         parentInWorldInv.invert();
         RigidBodyTransform pelvisInParent = new RigidBodyTransform();
         pelvisInParent.set(parentInWorldInv);
         pelvisInParent.multiply(pelvisInWorld); // pelvis in parent frame

         // 3. Camera transform in world (from sensor)
         RigidBodyTransform cameraInWorld = sensor.getTrackedSensorFrame().getTransformToRoot();
         // Apply stored initial World frame offset
         RigidBodyTransform correctedCameraInWorld = new RigidBodyTransform();
         correctedCameraInWorld.set(cameraInWorld);
         correctedCameraInWorld.multiply(initialCameraWorldOffset);

         // 4. Chain: correctedCameraInWorld → parentInCamera → pelvisInParent
         RigidBodyTransform pelvisInWorldEstimated = new RigidBodyTransform();
         pelvisInWorldEstimated.set(correctedCameraInWorld);
         pelvisInWorldEstimated.multiply(parentInCamera);
         pelvisInWorldEstimated.multiply(pelvisInParent);

         StampedPosePacket newestStampedPosePacket = new StampedPosePacket();
         newestStampedPosePacket.getPose().set(pelvisInWorldEstimated);
         newestStampedPosePacket.setTimestamp(syncedRobot.getTimestamp());
         odometryCorrectionPublisher.publish(newestStampedPosePacket);

         StampedOdometryPacket newestStampedOdometryPacket = new StampedOdometryPacket();
         newestStampedOdometryPacket.getPose().set(pelvisInWorldEstimated);
         newestStampedOdometryPacket.setTimestamp(sensor.getLastGrabTimestamp());
         newestStampedOdometryPacket.getImuOrientation().set(sensor.getImuOrientation());
         odometryInfoPublisher.publish(newestStampedOdometryPacket);
      }
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();
   }
}