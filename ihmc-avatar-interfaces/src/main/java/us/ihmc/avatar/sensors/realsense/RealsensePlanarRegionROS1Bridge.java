package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RealsensePlanarRegionROS1Bridge
{
   private static final double pelvisToMountOrigin = 0.19;
   private static final double depthOffsetX = 0.058611;
   private static final double depthOffsetZ = 0.01;
   private static final double depthPitchingAngle = 70.0 / 180.0 * Math.PI;
   private static final double depthRollOffset = Math.toRadians(-0.5);
   private static final double depthThickness = 0.0245;
   public static final RigidBodyTransform transformPelvisToDepthCamera = new RigidBodyTransform();
   static
   {
      transformPelvisToDepthCamera.appendTranslation(pelvisToMountOrigin, 0.0, 0.0);
      transformPelvisToDepthCamera.appendTranslation(depthOffsetX, 0.0, depthOffsetZ);
      transformPelvisToDepthCamera.appendRollRotation(depthRollOffset);
      transformPelvisToDepthCamera.appendPitchRotation(depthPitchingAngle);
      transformPelvisToDepthCamera.appendTranslation(depthThickness, 0.0, 0.0);

      transformPelvisToDepthCamera.appendYawRotation(-Math.PI / 2);
      transformPelvisToDepthCamera.appendRollRotation(-Math.PI / 2);
   }

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;
   private final RemoteSyncedRobotModel syncedRobot;

   public RealsensePlanarRegionROS1Bridge(DRCRobotModel robotModel,
                                          RosMainNode ros1Node,
                                          ROS2NodeInterface ros2Node,
                                          String ros1InputTopic,
                                          ROS2Topic<PlanarRegionsListMessage> ros2OutputTopic)
   {
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);

      ros1Node.attachSubscriber(ros1InputTopic, new AbstractRosTopicSubscriber<RawGPUPlanarRegionList>(RawGPUPlanarRegionList._TYPE)
      {
         @Override
         public void onNewMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
         {
            acceptMessage(rawGPUPlanarRegionList);
         }
      });

      publisher = ROS2Tools.createPublisher(ros2Node, ros2OutputTopic);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
   }

   private void acceptMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      executorService.clearTaskQueue();
      executorService.execute(() ->
      {
         syncedRobot.update();

         PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
         planarRegionsList.applyTransform(transformPelvisToDepthCamera);
         publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      });
   }
}
