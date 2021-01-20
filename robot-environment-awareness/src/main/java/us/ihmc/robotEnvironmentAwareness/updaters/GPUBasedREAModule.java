package us.ihmc.robotEnvironmentAwareness.updaters;

import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.GPUModuleAPI;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RawGPUPlanarRegionSubscriber;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class GPUBasedREAModule implements PerceptionModule {

    private static final int THREAD_PERIOD_MILLISECONDS = 1000;

    private final Messager messager;
    private final ROS2Node ros2Node;
    private final RosMainNode rosMainNode;

    private RawGPUPlanarRegionList rawGPUPlanarRegionList;
    private ScheduledFuture<?> scheduled;
    private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

    private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
    private final RawGPUPlanarRegionSubscriber gpuPlanarRegionSubscriber = new RawGPUPlanarRegionSubscriber();


    public GPUBasedREAModule(Messager messager, ROS2Node ros2Node, RosMainNode rosMainNode){
        this.rosMainNode = rosMainNode;
        this.ros2Node = ros2Node;
        this.messager = messager;
        rosMainNode.attachSubscriber("/map/regions/test", gpuPlanarRegionSubscriber);
    }

    void mainUpdate(){
        LogTools.info("Regions Available:{}", gpuPlanarRegionSubscriber.regionListIsAvailable());
        if(gpuPlanarRegionSubscriber.regionListIsAvailable()){
            this.rawGPUPlanarRegionList = gpuPlanarRegionSubscriber.getRawPlanarRegionList();
            PlanarRegionsList regionList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
            LogTools.info("Raw:{} Generated:{}", rawGPUPlanarRegionList.getNumOfRegions(), regionList.getNumberOfPlanarRegions());

            messager.submitMessage(GPUModuleAPI.PlanarRegionData, regionList);
        }
    }

    @Override
    public void start() {

        if (scheduled == null)
        {
            scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
        }
    }

    @Override
    public void stop() {

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

    public static GPUBasedREAModule createIntraprocess(Messager messager, ROS2Node ros2Node, RosMainNode rosMainNode) throws URISyntaxException {
        return new GPUBasedREAModule(messager, ros2Node, rosMainNode);
    }
}
