package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedBufferedRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshot;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;

public class MapSensePlanarRegionROS1Bridge
{
   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;
   private final RemoteSyncedBufferedRobotModel syncedRobot;
   private final ReferenceFrame sensorFrame;
   private final Timer throttleTimer = new Timer();

   public MapSensePlanarRegionROS1Bridge(DRCRobotModel robotModel, RosMainNode ros1Node, ROS2NodeInterface ros2Node)
   {
      syncedRobot = new RemoteSyncedBufferedRobotModel(robotModel, ros2Node);
      RigidBodyTransformReadOnly frameToSensorTransform = robotModel.getSensorInformation().getSteppingCameraTransform();
      ReferenceFrame frameOfLinkThatSensorIsMountedTo = robotModel.getSensorInformation().getSteppingCameraFrame(syncedRobot.getReferenceFrames());
      sensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("l515", frameOfLinkThatSensorIsMountedTo, frameToSensorTransform);

      MapsenseTools.createROS1Callback(ros1Node, this::acceptMessage);



      publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.MAPSENSE_REGIONS);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);

      throttleTimer.reset();
   }

   private void acceptMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
//      if (throttleTimer.isExpired(UnitConversions.hertzToSeconds(10.0)))
      {
//         throttleTimer.reset();
         executorService.clearQueueAndExecute(() ->
         {
            TimerSnapshot dataReceptionTimerSnapshot = syncedRobot.getDataReceptionTimerSnapshot();
            if (!dataReceptionTimerSnapshot.isRunning(2.0))
            {
               LogTools.info("No robot data in {} s", dataReceptionTimerSnapshot.getTimePassedSinceReset());
            }
            else
            {
               syncedRobot.updateToBuffered(rawGPUPlanarRegionList.getHeader().getStamp().totalNsecs());

               PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
               planarRegionsList.applyTransform(MapsenseTools.getTransformFromCameraToWorld());
               planarRegionsList.applyTransform(sensorFrame.getTransformToWorldFrame());
               publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
            }
         });
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }
}
