package us.ihmc.perception.odometry;

import ihmc_common_msgs.msg.dds.StampedOdometryPacket;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class ZEDOdometryPublisherThread extends RepeatingTaskThread
{
   private final ZEDImageSensor zedSensor;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Publisher<StampedOdometryPacket> odometryPublisher;
   private volatile boolean initialPoseReceived = false;
   
   public ZEDOdometryPublisherThread(ROS2Node ros2Node, ZEDImageSensor zedSensor, ROS2SyncedRobotModel syncedRobot)
   {
      super(zedSensor.getSensorName() + "OdometryPublishThread");
      this.zedSensor = zedSensor;
      this.syncedRobot = syncedRobot;
      this.odometryPublisher = ros2Node.createPublisher(StateEstimatorAPI.getTopic(StampedOdometryPacket.class, syncedRobot.getRobotModel().getSimpleRobotName().toLowerCase()));
   }

   @Override
   protected void runTask()
   {
      while (!initialPoseReceived) {
         return;
      }

      long timestamp = zedSensor.getLastGrabTimestamp();
      // Get the pose of ZED in world frame
      RigidBodyTransform zedInWorld = zedSensor.getTrackedSensorFrame().getTransformToRoot();

//      // Get the transform of ZED from torso (torso → ZED)
//      RigidBodyTransform zedInTorso = syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform();
//
//      // Compute pelvis-in-torso using RigidBodyTransform
//      RigidBodyTransform chestInWorldInv = new RigidBodyTransform(syncedRobot.getFullRobotModel()
//                                                                            .getChest().getParentJoint().getFrameAfterJoint().getTransformToRoot());
//      chestInWorldInv.invert();
//
//      RigidBodyTransform pelvisInTorso = new RigidBodyTransform();
//      pelvisInTorso.set(chestInWorldInv);
//      pelvisInTorso.multiply(syncedRobot.getFullRobotModel().getPelvis().getParentJoint().getFrameAfterJoint().getTransformToRoot());
//
//      // Invert zedFromTorso to get torsoInZed (zed → torso)
//      RigidBodyTransform torsoInZed = new RigidBodyTransform(zedInTorso);
//      torsoInZed.invert();

      // Compose final world → zed → torso → pelvis transform
      RigidBodyTransform estimatedPelvisInWorld = new RigidBodyTransform();
      estimatedPelvisInWorld.set(zedInWorld);
//      estimatedPelvisInWorld.multiply(torsoInZed);
//      estimatedPelvisInWorld.multiply(pelvisInTorso);

      StampedOdometryPacket newestStampedOdometryPacket = new StampedOdometryPacket();
      newestStampedOdometryPacket.getPose().set(estimatedPelvisInWorld);
      newestStampedOdometryPacket.setTimestamp(timestamp);
      newestStampedOdometryPacket.getImuOrientation().set(zedSensor.getImuOrientation());
      odometryPublisher.publish(newestStampedOdometryPacket);
   }
   
   @Override
   public void kill()
   {
      super.kill();
      interrupt();
   }
}