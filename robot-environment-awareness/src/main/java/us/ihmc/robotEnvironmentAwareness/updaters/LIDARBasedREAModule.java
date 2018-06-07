package us.ihmc.robotEnvironmentAwareness.updaters;

import java.io.File;
import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LidarScanMessagePubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.tools.JOctoMapTools;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationKryoNetClassLists;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.ros2.Ros2Node;

public class LIDARBasedREAModule
{
   private static final String lidarScanTopicName = "/ihmc/lidar_scan";
   private static final String planarRegionsTopicName = "/ihmc/planar_regions";
   private static final String planarRegionRequestTopicName = "/ihmc/planar_region_request";

   private static final String ocTreeTimeReport = "OcTree update took: ";
   private static final String reportOcTreeStateTimeReport = "Reporting OcTree state took: ";
   private static final String planarRegionsTimeReport = "OcTreePlanarRegion update took: ";
   private static final String reportPlanarRegionsStateTimeReport = "Reporting Planar Regions state took: ";

   private final TimeReporter timeReporter = new TimeReporter(this);

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double OCTREE_RESOLUTION = 0.02;
   protected static final boolean DEBUG = true;

   private final Ros2Node ros2Node = new Ros2Node(PubSubImplementation.FAST_RTPS, "REA_module");

   private final NormalOcTree mainOctree = new NormalOcTree(OCTREE_RESOLUTION);

   private final REAOcTreeBuffer bufferUpdater;
   private final REAOcTreeUpdater mainUpdater;
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAModuleStateReporter moduleStateReporter;
   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   private final AtomicReference<Boolean> clearOcTree;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;

   private LIDARBasedREAModule(Messager reaMessager, File configurationFile) throws IOException
   {
      this.reaMessager = reaMessager;

      moduleStateReporter = new REAModuleStateReporter(reaMessager);
      bufferUpdater = new REAOcTreeBuffer(mainOctree.getResolution(), reaMessager, moduleStateReporter);
      mainUpdater = new REAOcTreeUpdater(mainOctree, bufferUpdater, reaMessager);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(mainOctree, reaMessager);

      ros2Node.createSubscription(new LidarScanMessagePubSubType(), this::dispatchLidarScanMessage, lidarScanTopicName);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> bufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration, (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(planarRegionFeatureUpdater, ros2Node, planarRegionsTopicName, planarRegionRequestTopicName);
      clearOcTree = reaMessager.createInput(REAModuleAPI.OcTreeClear, false);
   }

   private void dispatchLidarScanMessage(Subscriber<LidarScanMessage> subscriber)
   {
      LidarScanMessage message = subscriber.takeNextData();
      moduleStateReporter.registerLidarScanMessage(message);
      bufferUpdater.handleLidarScanMessage(message);
      mainUpdater.handleLidarScanMessage(message);
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      bufferUpdater.loadConfiguration(filePropertyHelper);
      mainUpdater.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private final AtomicDouble lastCompleteUpdate = new AtomicDouble(Double.NaN);

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      double currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      boolean ocTreeUpdateSuccess = true;

      try
      {
         if (clearOcTree.getAndSet(false))
         {
            bufferUpdater.clearBuffer();
            mainUpdater.clearOcTree();
            planarRegionFeatureUpdater.clearOcTree();
         }
         else
         {
            timeReporter.run(mainUpdater::update, ocTreeTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportOcTreeState(mainOctree), reportOcTreeStateTimeReport);

            if (isThreadInterrupted())
               return;

            timeReporter.run(planarRegionFeatureUpdater::update, planarRegionsTimeReport);
            timeReporter.run(() -> moduleStateReporter.reportPlanarRegionsState(planarRegionFeatureUpdater), reportPlanarRegionsStateTimeReport);

            planarRegionNetworkProvider.update(ocTreeUpdateSuccess);
         }

         if (isThreadInterrupted())
            return;

         if (planarRegionNetworkProvider.pollClearRequest())
            clearOcTree.set(true);
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
         else
         {
            PrintTools.error(LIDARBasedREAModule.class, e.getClass().getSimpleName());
         }
      }

      currentTime = JOctoMapTools.nanoSecondsToSeconds(System.nanoTime());

      if (ocTreeUpdateSuccess)
         lastCompleteUpdate.set(currentTime);
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start() throws IOException
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(bufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      PrintTools.info("REA Module is going down.");

      reaMessager.closeMessager();

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

   public static LIDARBasedREAModule createRemoteModule(String configurationFilePath) throws Exception
   {
      Messager server = KryoMessager.createTCPServer(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                     REACommunicationKryoNetClassLists.getPrivateNetClassList());
      server.startMessager();
      return new LIDARBasedREAModule(server, new File(configurationFilePath));
   }

   public static LIDARBasedREAModule createIntraprocessModule(String configurationFilePath) throws Exception
   {
      Messager messager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                          REACommunicationKryoNetClassLists.getPrivateNetClassList());
      messager.startMessager();
      return new LIDARBasedREAModule(messager, new File(configurationFilePath));
   }
}
