package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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
   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;
   private final RemoteSyncedRobotModel syncedRobot;
   private final MovingReferenceFrame pelvisFrame;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private final RigidBodyTransform pelvisToSensorTransform;

   public RealsensePlanarRegionROS1Bridge(DRCRobotModel robotModel,
                                          RosMainNode ros1Node,
                                          ROS2NodeInterface ros2Node,
                                          String ros1InputTopic,
                                          ROS2Topic<PlanarRegionsListMessage> ros2OutputTopic,
                                          RigidBodyTransform pelvisToSensorTransform)
   {
      this.pelvisToSensorTransform = pelvisToSensorTransform;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();

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

         pelvisFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
         transformToWorld.multiply(pelvisToSensorTransform);

         PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
         planarRegionsList.applyTransform(transformToWorld);
         publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      });
   }
}
