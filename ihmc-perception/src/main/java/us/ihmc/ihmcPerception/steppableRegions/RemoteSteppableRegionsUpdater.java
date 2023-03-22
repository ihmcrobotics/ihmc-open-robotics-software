package us.ihmc.ihmcPerception.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ihmcPerception.heightMap.HeightMapAPI;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.ros2.ROS2Node;
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
import java.util.function.Supplier;

public class RemoteSteppableRegionsUpdater
{
   private static final long updateDTMillis = 200;

   private final SteppableRegionsUpdater steppableRegionsUpdater;
   private final Supplier<Boolean> computeSteppableRegions;

   private final SteppableRegionCalculatorParameters steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()),
                                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public RemoteSteppableRegionsUpdater(ROS2PublishSubscribeAPI rosHelper, SteppableRegionCalculatorParametersReadOnly defaultParameters)
   {
      this(rosHelper, defaultParameters, () -> true);
   }

   public RemoteSteppableRegionsUpdater(ROS2PublishSubscribeAPI rosHelper,
                                        SteppableRegionCalculatorParametersReadOnly defaultParameters,
                                        Supplier<Boolean> computeSteppableRegions)
   {
      this.computeSteppableRegions = computeSteppableRegions;
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(rosHelper);
      ros2PropertySetGroup.registerStoredPropertySet(SteppableRegionsAPI.PARAMETERS, steppableRegionCalculatorParameters);

      rosHelper.createPublisher(SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT);
      rosHelper.createPublisher(SteppableRegionsAPI.STEPPABLE_REGIONS_DEBUG_OUTPUT);

      steppableRegionsUpdater = new SteppableRegionsUpdater(defaultParameters);
      steppableRegionsUpdater.addSteppableRegionListCollectionOutputConsumer(message -> rosHelper.publish(SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT, message));
      steppableRegionsUpdater.addSteppableRegionDebugConsumer(message -> rosHelper.publish(SteppableRegionsAPI.STEPPABLE_REGIONS_DEBUG_OUTPUT, message));
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

      if (computeSteppableRegions.get())
      {
         steppableRegionsUpdater.submitLatestSteppableRegionCalculatorParameters(steppableRegionCalculatorParameters);
         steppableRegionsUpdater.compute();
      }
   }

   public void destroy()
   {
      executorService.shutdown();
   }
}
