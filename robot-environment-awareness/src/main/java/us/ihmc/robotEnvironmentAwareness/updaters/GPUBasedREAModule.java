package us.ihmc.robotEnvironmentAwareness.updaters;

import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
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

    private static final int THREAD_PERIOD_MILLISECONDS = 200;

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
        if(gpuPlanarRegionSubscriber.regionListIsAvailable()){
            this.rawGPUPlanarRegionList = gpuPlanarRegionSubscriber.getPlanarRegionList();
            for (int i = 0; i < rawGPUPlanarRegionList.getNumOfRegions(); i++) {
                List<RawGPUPlanarRegion> regions = rawGPUPlanarRegionList.getRegions();
                System.out.println("\tRegion:" + regions.get(i).getId());
                System.out.println("\t\tNormal:" + "X:" + regions.get(i).getNormal().getX() + "\tY:" + regions.get(i).getNormal().getY() + "\tZ:" + regions.get(i).getNormal().getZ());
                System.out.println("\t\tCentroid:" + "X:" + regions.get(i).getCentroid().getX() + "\tY:" + regions.get(i).getCentroid().getY() + "\tZ:" + regions.get(i).getCentroid().getZ());
//            for (int j = 0; j < regions.get(i).getVertices().size(); j++) {
//                geometry_msgs.Point point = regions.get(i).getVertices().get(j);
//                System.out.println("\t\tPoint:" + "X:" + point.getX() + "\tY:" + point.getY() + "\tZ:" + point.getZ());
//            }
            }
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
