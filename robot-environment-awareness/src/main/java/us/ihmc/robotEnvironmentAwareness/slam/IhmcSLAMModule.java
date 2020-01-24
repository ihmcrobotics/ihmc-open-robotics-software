package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.IOException;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMModule
{
   private final Messager reaMessager;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   private final AtomicReference<Boolean> enable;

   private final Topic<PlanarRegionsListMessage> planarRegionsStateTopicToSubmit;
   private final AtomicReference<StereoVisionPointCloudMessage> newPointCloud = new AtomicReference<>(null);
   private final LinkedList<StereoVisionPointCloudMessage> stackedPointCloud = new LinkedList<StereoVisionPointCloudMessage>();

   private final RandomICPSLAM slam = new RandomICPSLAM(DEFAULT_OCTREE_RESOLUTION);

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private static final int THREAD_PERIOD_MILLISECONDS = 1;
   private ScheduledFuture<?> scheduledMain;
   private ScheduledFuture<?> scheduledSLAM;

   public IhmcSLAMModule(Messager reaMessager, Topic<Boolean> slamEnableTopic, Topic<PlanarRegionsListMessage> slamPlanarRegionsStateTopic)
   {
      this.reaMessager = reaMessager;
      enable = reaMessager.createInput(slamEnableTopic, true);
      planarRegionsStateTopicToSubmit = slamPlanarRegionsStateTopic;
   }

   public void start() throws IOException
   {
      if (scheduledMain == null)
      {
         scheduledMain = executorService.scheduleAtFixedRate(this::updateMain, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }

      if (scheduledSLAM == null)
      {
         scheduledSLAM = executorService.scheduleAtFixedRate(this::updateSLAM, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      LogTools.info("IhmcSLAMModule is going down.");

      reaMessager.closeMessager();

      if (scheduledMain != null)
      {
         scheduledMain.cancel(true);
         scheduledMain = null;
      }

      if (scheduledSLAM != null)
      {
         scheduledSLAM.cancel(true);
         scheduledSLAM = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   private boolean isMainThreadInterrupted()
   {
      return Thread.interrupted() || scheduledMain == null || scheduledMain.isCancelled();
   }

   private boolean isSLAMThreadInterrupted()
   {
      return Thread.interrupted() || scheduledSLAM == null || scheduledSLAM.isCancelled();
   }

   public void updateSLAM()
   {
      if (isSLAMThreadInterrupted())
         return;

      if (stackedPointCloud.size() == 0)
         return;

      StereoVisionPointCloudMessage pointCloudToCompute = stackedPointCloud.getFirst();

      System.out.println("stacked point cloud data set = [" + stackedPointCloud.size() + "].");
      if (slam.isEmpty())
      {
         System.out.println("add first frame.");
         slam.addFirstFrame(pointCloudToCompute);
         //reaMessager.submitMessage(REAModuleAPI.IhmcSLAMFrameState, slam.getLatestFrame());
      }
      else
      {
         System.out.println("add frame [" + stackedPointCloud.size() + "].");
         boolean merged = slam.addFrame(pointCloudToCompute);
         //         if (merged)
         //            reaMessager.submitMessage(REAModuleAPI.IhmcSLAMFrameState, slam.getLatestFrame());
      }
      stackedPointCloud.removeFirst();
      System.out.println("SLAM Computation is done [" + stackedPointCloud.size() + "].");
   }

   public void updateMain()
   {
      if (isMainThreadInterrupted())
         return;

      if (enable.get())
      {
         StereoVisionPointCloudMessage pointCloud = newPointCloud.getAndSet(null);
         if (pointCloud == null)
            return;

         stackedPointCloud.add(pointCloud);
         System.out.println("New point cloud is stacked [" + stackedPointCloud.size() + "].");
      }
   }

   public void clearSLAM()
   {
      stackedPointCloud.clear();
      slam.clear();
      newPointCloud.set(null);
   }

   public void handlePointCloud(StereoVisionPointCloudMessage message)
   {
      newPointCloud.set(message);
   }

   public void buildAndSubmitPlanarRegionsMap()
   {
      slam.updatePlanarRegionsMap();
      PlanarRegionsList planarRegionsMap = slam.getPlanarRegionsMap();
      reaMessager.submitMessage(planarRegionsStateTopicToSubmit, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsMap));
   }
}
