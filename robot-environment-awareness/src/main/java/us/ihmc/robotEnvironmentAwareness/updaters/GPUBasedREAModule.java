package us.ihmc.robotEnvironmentAwareness.updaters;

import map_sense.RawGPUPlanarRegionList;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.GPUPerceptionModuleAPI;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.RawGPUPlanarRegionSubscriber;

import java.net.URISyntaxException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class GPUBasedREAModule implements PerceptionModule
{

   private static final int THREAD_PERIOD_MILLISECONDS = 100;

   private final Messager messager;
   private final ROS2Node ros2Node;
   private final RosMainNode rosMainNode;

   private RawGPUPlanarRegionList rawGPUPlanarRegionList;
   private ScheduledFuture<?> scheduled;
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                  getClass(),
                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private final RawGPUPlanarRegionSubscriber gpuPlanarRegionSubscriber = new RawGPUPlanarRegionSubscriber();

   public GPUBasedREAModule(Messager messager, ROS2Node ros2Node, RosMainNode rosMainNode)
   {
      this.rosMainNode = rosMainNode;
      this.ros2Node = ros2Node;
      this.messager = messager;
      rosMainNode.attachSubscriber(RosTools.MAPSENSE_REGIONS, gpuPlanarRegionSubscriber);
   }

   void mainUpdate()
   {
      if (gpuPlanarRegionSubscriber.regionListIsAvailable())
      {
         this.rawGPUPlanarRegionList = gpuPlanarRegionSubscriber.getRawPlanarRegionList();
         PlanarRegionsList regionList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
         //            LogTools.info("Raw:{} Generated:{}", rawGPUPlanarRegionList.getNumOfRegions(), regionList.getNumberOfPlanarRegions());

         messager.submitMessage(GPUPerceptionModuleAPI.PlanarRegionData, regionList);
      }
   }

   @Override
   public void start()
   {

      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   @Override
   public void stop()
   {

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public static GPUBasedREAModule createIntraprocess(Messager messager, ROS2Node ros2Node, RosMainNode rosMainNode) throws URISyntaxException
   {
      return new GPUBasedREAModule(messager, ros2Node, rosMainNode);
   }
}
