package us.ihmc.ihmcPerception.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.ihmcPerception.heightMap.HeightMapAPI;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class RemoteSteppableRegionsUpdater
{
   private static final long updateDTMillis = 100;

   private final RealtimeROS2Node ros2Node;
   private final SteppableRegionsUpdater steppableRegionsUpdater = new SteppableRegionsUpdater();

   private final SteppableRegionCalculatorParameters steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public RemoteSteppableRegionsUpdater(RealtimeROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(new ROS2Helper(ros2Node));
      ros2PropertySetGroup.registerStoredPropertySet(SteppableRegionsAPI.PARAMETERS, steppableRegionCalculatorParameters);

      IHMCRealtimeROS2Publisher<SteppableRegionsListCollectionMessage> steppableRegionsPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                                             SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT);
      IHMCRealtimeROS2Publisher<SteppableRegionDebugImagesMessage> steppableRegionsDebugPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                                              SteppableRegionsAPI.STEPPABLE_REGIONS_DEBUG_OUTPUT);

      steppableRegionsUpdater.addSteppableRegionListCollectionOutputConsumer(steppableRegionsPublisher::publish);
      steppableRegionsUpdater.addSteppableRegionDebugConsumer(steppableRegionsDebugPublisher::publish);
   }

   public void submitLatestHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      this.steppableRegionsUpdater.submitLatestHeightMapMessage(heightMapMessage);
   }

   public void start()
   {
      executorService.scheduleAtFixedRate(this::compute, 0, updateDTMillis, TimeUnit.MILLISECONDS);
   }

   public void compute()
   {
      ros2PropertySetGroup.update();

      steppableRegionsUpdater.submitLatestSteppableRegionCalculatorParameters(steppableRegionCalculatorParameters);
      steppableRegionsUpdater.compute();
   }

   public void destroy()
   {
      ros2Node.destroy();
      executorService.shutdown();
   }
}
