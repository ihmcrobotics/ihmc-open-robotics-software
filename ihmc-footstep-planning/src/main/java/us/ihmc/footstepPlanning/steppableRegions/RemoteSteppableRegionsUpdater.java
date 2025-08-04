package us.ihmc.footstepPlanning.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public class RemoteSteppableRegionsUpdater
{
   private static final long updateDTMillis = 200;

   private final SteppableRegionsUpdater steppableRegionsUpdater;
   private final Supplier<Boolean> computeSteppableRegions;

   private final SteppableRegionCalculatorParameters steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;

   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;

   public RemoteSteppableRegionsUpdater(ROS2Node ros2Node, SteppableRegionCalculatorParametersReadOnly defaultParameters)
   {
      this(ros2Node, defaultParameters, () -> true);
   }

   public RemoteSteppableRegionsUpdater(ROS2Node ros2Node,
                                        SteppableRegionCalculatorParametersReadOnly defaultParameters,
                                        Supplier<Boolean> computeSteppableRegions)
   {
      this.computeSteppableRegions = computeSteppableRegions;
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);
      ros2PropertySetGroup.registerStoredPropertySet(SteppableRegionsAPI.PARAMETERS, steppableRegionCalculatorParameters);

      ROS2Publisher<SteppableRegionsListCollectionMessage> steppableRegionsOutputPublisher = ros2Node.createPublisher(SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT);
      ROS2Publisher<SteppableRegionDebugImagesMessage> steppableRegionsDebugOutputPublisher = ros2Node.createPublisher(SteppableRegionsAPI.STEPPABLE_REGIONS_DEBUG_OUTPUT);

      steppableRegionsUpdater = new SteppableRegionsUpdater(defaultParameters);
      steppableRegionsUpdater.addSteppableRegionListCollectionOutputConsumer(steppableRegionsOutputPublisher::publish);
      steppableRegionsUpdater.addSteppableRegionDebugConsumer(steppableRegionsDebugOutputPublisher::publish);
   }

   public void submitLatestHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      this.steppableRegionsUpdater.submitLatestHeightMapMessage(heightMapMessage);
   }

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::compute, 0, updateDTMillis, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }
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
      stop();
      executorService.shutdown();
   }
}
