package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.stage.Stage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.*;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionModule;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

public class LiveMapModule implements PerceptionModule
{
   private static final int THREAD_PERIOD_MILLISECONDS = 200;

   private final Ros2Node ros2Node;
   private final Messager messager;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;

   private final AtomicReference<PlanarRegionsListMessage> mostRecentLocalizedMap;
   private final AtomicReference<PlanarRegionsListMessage> mostRecentRegionsAtFeet;

   private final AtomicBoolean hasNewLocalizedMap = new AtomicBoolean(false);
   private final AtomicBoolean hasNewRegionsAtFeet = new AtomicBoolean(false);

   private final AtomicReference<Boolean> viewingEnabled;
   private final AtomicReference<PlanarRegionsListMessage> combinedLiveMap;

   private final AtomicReference<Boolean> enableLidar;
   private final AtomicReference<Boolean> enableRealSense;

   private final AtomicReference<Boolean> clearLidar;
   private final AtomicReference<Boolean> clearRealSense;

   private final AtomicReference<PlanarRegionSLAMParameters> slamParameters;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> combinedMapPublisher;

   private LiveMapModule(Ros2Node ros2Node, Messager messager)
   {
      this.ros2Node = ros2Node;
      this.messager = messager;

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_MAP, this::dispatchLocalizedMap);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, stereoOutputTopic, this::dispatchRegionsAtFeet);

      mostRecentLocalizedMap = messager.createInput(LiveMapModuleAPI.LocalizedMap, null);
      mostRecentRegionsAtFeet = messager.createInput(LiveMapModuleAPI.RegionsAtFeet, null);

      slamParameters = messager.createInput(LiveMapModuleAPI.PlanarRegionsSLAMParameters, new PlanarRegionSLAMParameters());

      messager.registerTopicListener(LiveMapModuleAPI.LocalizedMap, (message) -> hasNewLocalizedMap.set(true));
      messager.registerTopicListener(LiveMapModuleAPI.RegionsAtFeet, (message) -> hasNewRegionsAtFeet.set(true));

      viewingEnabled = messager.createInput(LiveMapModuleAPI.ViewingEnable, true);
      combinedLiveMap = messager.createInput(LiveMapModuleAPI.CombinedLiveMap);

      enableRealSense = messager.createInput(LiveMapModuleAPI.EnableRealSense, true);
      enableLidar = messager.createInput(LiveMapModuleAPI.EnableLidar, false);

      clearRealSense = messager.createInput(LiveMapModuleAPI.ClearRealSense, false);
      clearLidar = messager.createInput(LiveMapModuleAPI.ClearLidar, false);

      messager.registerTopicListener(LiveMapModuleAPI.RequestEntireModuleState, request -> sendCurrentState());

      sendCurrentState();

      // FIXME fix the output topic name.
      combinedMapPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, outputTopic);
   }

   private void sendCurrentState()
   {
      messager.submitMessage(LiveMapModuleAPI.EnableLidar, enableLidar.get());
      messager.submitMessage(LiveMapModuleAPI.EnableRealSense, enableRealSense.get());

      messager.submitMessage(LiveMapModuleAPI.ViewingEnable, viewingEnabled.get());
      messager.submitMessage(LiveMapModuleAPI.CombinedLiveMap, combinedLiveMap.get());
   }

   private void dispatchLocalizedMap(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapModuleAPI.LocalizedMap, message);
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

      if (clearLidar.getAndSet(false))
      {
         // TODO
      }
      if (clearRealSense.getAndSet(false))
      {
         hasNewRegionsAtFeet.set(false);
         mostRecentRegionsAtFeet.set(null);
      }

      boolean shouldUpdateMap = hasNewLocalizedMap.getAndSet(false) || hasNewRegionsAtFeet.getAndSet(false); // TODO || hasNewLidarMap.getAndSet(false);

      if (shouldUpdateMap)
      {
         PlanarRegionsList localizedMap = PlanarRegionMessageConverter.convertToPlanarRegionsList(mostRecentLocalizedMap.get());

         PlanarRegionsList regionsAtFeet = PlanarRegionMessageConverter.convertToPlanarRegionsList(mostRecentRegionsAtFeet.get());
         localizedMap = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(localizedMap, regionsAtFeet, slamParameters.get(), null);

         PlanarRegionsListMessage mapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(localizedMap);
         messager.submitMessage(LiveMapModuleAPI.CombinedLiveMap, mapMessage);
         combinedMapPublisher.publish(mapMessage);
      }
   }

   public static LiveMapModule createIntraprocess(Ros2Node ros2Node) throws Exception
   {
      Messager messager = createKryoMessager();
      return new LiveMapModule(ros2Node, messager);
   }

   private static Messager createKryoMessager() throws Exception
   {
      KryoMessager messager = KryoMessager.createIntraprocess(LiveMapModuleAPI.API,
                                                              NetworkPorts.LIVEMAP_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();
      return messager;
   }
}
