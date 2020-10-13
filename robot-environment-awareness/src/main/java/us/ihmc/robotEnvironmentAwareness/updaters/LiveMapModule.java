package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.*;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class LiveMapModule implements PerceptionModule
{
   private static final int THREAD_PERIOD_MILLISECONDS = 200;

   private final ROS2Node ros2Node;
   private final Messager messager;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;

   private final AtomicReference<PlanarRegionsListMessage> mostRecentLocalizedMap;
   private final AtomicReference<PlanarRegionsListMessage> mostRecentRegionsAtFeet;
   private final AtomicReference<PlanarRegionsListMessage> mostRecentLidarMap;

   private final AtomicBoolean hasNewLocalizedMap = new AtomicBoolean(false);
   private final AtomicBoolean hasNewRegionsAtFeet = new AtomicBoolean(false);
   private final AtomicBoolean hasNewLidarMap = new AtomicBoolean(false);
   private final AtomicBoolean hasNewParameters = new AtomicBoolean(false);

   private final AtomicReference<Boolean> viewingEnabled;
   private final AtomicReference<PlanarRegionsListMessage> combinedLiveMap;

   private final AtomicReference<Boolean> enableMapFusion;
   private final AtomicReference<Boolean> enableLidar;
   private final AtomicReference<Boolean> enableRealSense;

   private final AtomicReference<Boolean> clearLidar;
   private final AtomicReference<Boolean> clearRealSense;
   private final AtomicReference<Boolean> clearLocalizedMap;

   private final PlanarRegionSLAMParameters slamParameters;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> combinedMapPublisher;

   private LiveMapModule(ROS2Node ros2Node, Messager messager)
   {
      this(ros2Node, messager, null);
   }

   private LiveMapModule(ROS2Node ros2Node, Messager messager, String configurationFileProject)
   {
      this.ros2Node = ros2Node;
      this.messager = messager;

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_REGIONS, this::dispatchLocalizedMap);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, lidarOutputTopic, this::dispatchLidarMap);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, stereoOutputTopic, this::dispatchRegionsAtFeet);

      mostRecentLocalizedMap = messager.createInput(LiveMapModuleAPI.LocalizedMap, null);
      mostRecentRegionsAtFeet = messager.createInput(LiveMapModuleAPI.RegionsAtFeet, null);
      mostRecentLidarMap = messager.createInput(LiveMapModuleAPI.LidarMap, null);

      if (configurationFileProject != null)
         slamParameters = new PlanarRegionSLAMParameters("ForLiveMap", configurationFileProject);
      else
         slamParameters = new PlanarRegionSLAMParameters("ForLiveMap");

      messager.registerTopicListener(LiveMapModuleAPI.PlanarRegionsSLAMParameters, parameters ->
      {
         slamParameters.setAllFromStrings(parameters);
         hasNewParameters.set(true);
      });

      messager.registerTopicListener(LiveMapModuleAPI.LocalizedMap, (message) -> hasNewLocalizedMap.set(true));
      messager.registerTopicListener(LiveMapModuleAPI.RegionsAtFeet, (message) -> hasNewRegionsAtFeet.set(true));
      messager.registerTopicListener(LiveMapModuleAPI.LidarMap, (message) -> hasNewLidarMap.set(true));

      viewingEnabled = messager.createInput(LiveMapModuleAPI.ViewingEnable, true);
      combinedLiveMap = messager.createInput(LiveMapModuleAPI.CombinedLiveMap);

      enableMapFusion = messager.createInput(LiveMapModuleAPI.EnableMapFusion, true);
      enableRealSense = messager.createInput(LiveMapModuleAPI.EnableRealSense, true);
      enableLidar = messager.createInput(LiveMapModuleAPI.EnableLidar, false);

      clearRealSense = messager.createInput(LiveMapModuleAPI.ClearRealSense, false);
      clearLidar = messager.createInput(LiveMapModuleAPI.ClearLidar, false);
      clearLocalizedMap = messager.createInput(LiveMapModuleAPI.ClearLocalizedMap, false);

      messager.registerTopicListener(LiveMapModuleAPI.RequestEntireModuleState, request -> sendCurrentState());

      sendCurrentState();

      combinedMapPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.MAP_REGIONS);
   }

   private void sendCurrentState()
   {
      messager.submitMessage(LiveMapModuleAPI.EnableMapFusion, enableMapFusion.get());
      messager.submitMessage(LiveMapModuleAPI.EnableLidar, enableLidar.get());
      messager.submitMessage(LiveMapModuleAPI.EnableRealSense, enableRealSense.get());

      messager.submitMessage(LiveMapModuleAPI.ViewingEnable, viewingEnabled.get());
      messager.submitMessage(LiveMapModuleAPI.CombinedLiveMap, combinedLiveMap.get());
      messager.submitMessage(LiveMapModuleAPI.PlanarRegionsSLAMParameters, slamParameters.getAllAsStrings());
   }

   private void dispatchLocalizedMap(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapModuleAPI.LocalizedMap, message);
   }

   private void dispatchLidarMap(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapModuleAPI.LidarMap, message);
   }

   private void dispatchRegionsAtFeet(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapModuleAPI.RegionsAtFeet, message);
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      LogTools.info("Live Map Module is going down.");

      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      ros2Node.destroy();

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

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      boolean shouldUpdateMap = false;
      if (clearLocalizedMap.getAndSet(false))
      {
         shouldUpdateMap = true;
         hasNewLocalizedMap.set(false);
         mostRecentLocalizedMap.set(null);
      }
      if (clearLidar.getAndSet(false))
      {
         shouldUpdateMap = true;
         hasNewLidarMap.set(false);
         mostRecentLidarMap.set(null);
      }
      if (clearRealSense.getAndSet(false))
      {
         shouldUpdateMap = true;
         hasNewRegionsAtFeet.set(false);
         mostRecentRegionsAtFeet.set(null);
      }

      shouldUpdateMap |= hasNewLocalizedMap.get() || hasNewRegionsAtFeet.get() || hasNewLidarMap.get() || hasNewParameters.get();

      if (shouldUpdateMap)
      {
         PlanarRegionsList localizedMap = null;
         if (mostRecentLocalizedMap.get() != null)
         {
            localizedMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(mostRecentLocalizedMap.get());
            hasNewLocalizedMap.set(false);
         }

         if (enableMapFusion.get() && enableRealSense.get() && mostRecentRegionsAtFeet.get() != null)
         {
            PlanarRegionsList regionsToFuse = PlanarRegionMessageConverter.convertToPlanarRegionsList(mostRecentRegionsAtFeet.get());
            if (localizedMap != null)
               localizedMap = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(regionsToFuse, localizedMap, slamParameters, null);
            else
               localizedMap = regionsToFuse;

            hasNewParameters.set(false);
            hasNewRegionsAtFeet.set(false);
         }

         if (enableMapFusion.get() && enableLidar.get() && mostRecentLidarMap.get() != null)
         {
            PlanarRegionsList regionsToFuse = PlanarRegionMessageConverter.convertToPlanarRegionsList(mostRecentLidarMap.get());
            if (localizedMap != null)
               localizedMap = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(localizedMap, regionsToFuse, slamParameters, null);
            else
               localizedMap = regionsToFuse;

            hasNewParameters.set(false);
            hasNewLidarMap.set(false);
         }

         PlanarRegionsListMessage mapMessage;
         if (localizedMap != null)
            mapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(localizedMap);
         else
            mapMessage = null;

         messager.submitMessage(LiveMapModuleAPI.CombinedLiveMap, mapMessage);
         combinedMapPublisher.publish(mapMessage);
      }
   }

   public static LiveMapModule createIntraprocess(ROS2Node ros2Node, Messager messager)
   {
      return createIntraprocess(ros2Node, messager, null);
   }

   public static LiveMapModule createIntraprocess(ROS2Node ros2Node, Messager messager, String configurationFileProject)
   {
      return new LiveMapModule(ros2Node, messager, configurationFileProject);
   }
}
