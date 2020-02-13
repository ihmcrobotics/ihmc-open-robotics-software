package us.ihmc.robotEnvironmentAwareness.slam;

import java.io.IOException;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMModule
{
   private final Messager reaMessager;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   private static final Color LATEST_POINT_CLOUD_COLOR = Color.GREEN;
   private static final Color SOURCE_POINT_CLOUD_COLOR = Color.RED;

   private final AtomicReference<Boolean> enable;

   private final Topic<PlanarRegionsListMessage> planarRegionsStateTopicToSubmit;
   private final AtomicReference<StereoVisionPointCloudMessage> newPointCloud = new AtomicReference<>(null);
   private final LinkedList<StereoVisionPointCloudMessage> pointCloudQueue = new LinkedList<StereoVisionPointCloudMessage>();

   private final RandomICPSLAM slam = new RandomICPSLAM(DEFAULT_OCTREE_RESOLUTION);

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private static final int THREAD_PERIOD_MILLISECONDS = 1;
   private ScheduledFuture<?> scheduledMain;
   private ScheduledFuture<?> scheduledSLAM;

   private final AtomicReference<IhmcSLAMParameters> ihmcSLAMParameters;

   public IhmcSLAMModule(Messager reaMessager, Topic<Boolean> slamEnableTopic, Topic<PlanarRegionsListMessage> slamPlanarRegionsStateTopic)
   {
      this.reaMessager = reaMessager;
      enable = reaMessager.createInput(slamEnableTopic, true);
      planarRegionsStateTopicToSubmit = slamPlanarRegionsStateTopic;

      ihmcSLAMParameters = reaMessager.createInput(REAModuleAPI.SLAMParameters, new IhmcSLAMParameters());

      reaMessager.registerTopicListener(REAModuleAPI.SLAMClear, (content) -> clearSLAM());
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

      if (pointCloudQueue.size() == 0)
         return;

      updateSLAMParameters();

      StereoVisionPointCloudMessage pointCloudToCompute = pointCloudQueue.getFirst();

      String stringToReport = "";
      boolean success;
      if (slam.isEmpty())
      {
         slam.addFirstFrame(pointCloudToCompute);
         success = true;
      }
      else
      {
         success = slam.addFrame(pointCloudToCompute);
      }
      pointCloudQueue.removeFirst();
      reaMessager.submitMessage(REAModuleAPI.QueuedBuffers, pointCloudQueue.size() + " [" + slam.getPointCloudMap().size() + "]");
      stringToReport = stringToReport + success + " " + slam.getPointCloudMap().size() + " " + slam.getComputationTimeForLatestFrame() + " (sec) ";
      reaMessager.submitMessage(REAModuleAPI.SLAMStatus, stringToReport);

      if (success)
      {
         NormalOcTree octreeMap = slam.getOctree();
         NormalOcTreeMessage octreeMessage = OcTreeMessageConverter.convertToMessage(octreeMap);
         reaMessager.submitMessage(REAModuleAPI.SLAMOctreeMapState, octreeMessage);

         slam.updatePlanarRegionsMap();
         PlanarRegionsList planarRegionsMap = slam.getPlanarRegionsMap();
         reaMessager.submitMessage(planarRegionsStateTopicToSubmit, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsMap));

         // TODO:
         reaMessager.submitMessage(REAModuleAPI.SLAMSensorFrameState, new Pose3D());
      }

      Point3DReadOnly[] originalPointCloud = slam.getLatestOriginalPointCloud();
      Point3DReadOnly[] sourcePointsToWorld = slam.getSourcePointsToWorldLatestFrame();
      if (originalPointCloud == null || sourcePointsToWorld == null)
         return;
      StereoVisionPointCloudMessage latestStereoMessage = createLatestFrameStereoVisionPointCloudMessage(originalPointCloud, sourcePointsToWorld);
      reaMessager.submitMessage(REAModuleAPI.IhmcSLAMFrameState, latestStereoMessage);
   }

   private StereoVisionPointCloudMessage createLatestFrameStereoVisionPointCloudMessage(Point3DReadOnly[] originalPointCloud,
                                                                                        Point3DReadOnly[] sourcePointsToWorld)
   {
      int numberOfPointsToPack = originalPointCloud.length + sourcePointsToWorld.length;

      float[] pointCloudBuffer = new float[numberOfPointsToPack * 3];
      int[] colorBuffer = new int[numberOfPointsToPack * 3];
      for (int i = 0; i < originalPointCloud.length; i++)
      {
         pointCloudBuffer[3 * i + 0] = (float) originalPointCloud[i].getX();
         pointCloudBuffer[3 * i + 1] = (float) originalPointCloud[i].getY();
         pointCloudBuffer[3 * i + 2] = (float) originalPointCloud[i].getZ();
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(LATEST_POINT_CLOUD_COLOR);
      }
      for (int i = originalPointCloud.length; i < numberOfPointsToPack; i++)
      {
         pointCloudBuffer[3 * i + 0] = (float) sourcePointsToWorld[i - originalPointCloud.length].getX();
         pointCloudBuffer[3 * i + 1] = (float) sourcePointsToWorld[i - originalPointCloud.length].getY();
         pointCloudBuffer[3 * i + 2] = (float) sourcePointsToWorld[i - originalPointCloud.length].getZ();
         colorBuffer[i] = StereoVisionPointCloudViewer.colorToInt(SOURCE_POINT_CLOUD_COLOR);
      }
      return MessageTools.createStereoVisionPointCloudMessage(19870612L, pointCloudBuffer, colorBuffer);
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

         pointCloudQueue.add(pointCloud);
      }
   }

   private void updateSLAMParameters()
   {
      IhmcSLAMParameters parameters = ihmcSLAMParameters.get();
      slam.updateParameters(parameters);
   }

   public void clearSLAM()
   {
      pointCloudQueue.clear();
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
