package us.ihmc.robotEnvironmentAwareness.updaters;

import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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

    private static final int THREAD_PERIOD_MILLISECONDS = 100;

    private RosMainNode rosMainNode;
    private RawGPUPlanarRegionList rawGPUPlanarRegionList;
    private ScheduledFuture<?> scheduled;
    private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

    private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
    private final RawGPUPlanarRegionSubscriber gpuPlanarRegionSubscriber = new RawGPUPlanarRegionSubscriber();


    public GPUBasedREAModule() throws URISyntaxException {
        URI rosMasterURI = new URI("http://localhost:11311/");
        rosMainNode = new RosMainNode(rosMasterURI, "GPUPlanarRegionSubscriber");
        rosMainNode.attachSubscriber("/map/regions/test", gpuPlanarRegionSubscriber);
        rosMainNode.execute();
    }

    void mainUpdate(){
        LogTools.info("Regions Available:{}", gpuPlanarRegionSubscriber.regionListIsAvailable());
        if(gpuPlanarRegionSubscriber.regionListIsAvailable()){
            this.rawGPUPlanarRegionList = gpuPlanarRegionSubscriber.getRawPlanarRegionList();
            PlanarRegionsList regionList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
            LogTools.info("Raw:{} Generated:{}", rawGPUPlanarRegionList.getNumOfRegions(), regionList.getNumberOfPlanarRegions());
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
        if (rosMainNode.isStarted())
            rosMainNode.shutdown();

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
}
