package us.ihmc.avatar.sensors.realsense;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedBufferedRobotModel;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
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
   private final ROS2PublisherBasics<PlanarRegionsListMessage> publisher;
   private final ROS2SyncedBufferedRobotModel syncedRobot;
   private final Timer throttleTimer = new Timer();

   public MapSensePlanarRegionROS1Bridge(DRCRobotModel robotModel, RosMainNode ros1Node, ROS2NodeInterface ros2Node)
   {
      syncedRobot = new ROS2SyncedBufferedRobotModel(robotModel, ros2Node);

      MapsenseTools.createROS1Callback(ros1Node, this::acceptMessage);

      publisher = ROS2Tools.createPublisher(ros2Node, PerceptionAPI.MAPSENSE_REGIONS);

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
               planarRegionsList.applyTransform(syncedRobot.getReferenceFrames().getSteppingCameraFrame().getTransformToWorldFrame());
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
