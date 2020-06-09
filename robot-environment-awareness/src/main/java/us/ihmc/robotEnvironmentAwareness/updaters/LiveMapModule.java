package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;

public class LiveMapModule
{
   private static final int THREAD_PERIOD_MILLISECONDS = 200;

   private final Ros2Node ros2Node;
   private final Messager messager;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;

   private final AtomicReference<PlanarRegionsList> mostRecentLocalizedMap;
   private final AtomicReference<PlanarRegionsList> mostRecentRegionsAtFeet;

   private final AtomicBoolean hasNewLocalizedMap = new AtomicBoolean(false);
   private final AtomicBoolean hasNewRegionsAtFeet = new AtomicBoolean(false);

   private final AtomicReference<PlanarRegionSLAMParameters> slamParameters;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> combinedMapPublisher;

   private LiveMapModule(Ros2Node ros2Node, Messager messager)
   {
      this.ros2Node = ros2Node;
      this.messager = messager;

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REALSENSE_SLAM_MAP,
                                                    this::dispatchLocalizedMap);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, outputTopic,
                                                    this::dispatchRegionsAtFeet);

      mostRecentLocalizedMap = messager.createInput(LiveMapAPI.LocalizedMap, null);
      mostRecentRegionsAtFeet = messager.createInput(LiveMapAPI.RegionsAtFeet, null);

      slamParameters = messager.createInput(LiveMapAPI.PlanarRegionSLAMParameters, new PlanarRegionSLAMParameters());

      messager.registerTopicListener(LiveMapAPI.LocalizedMap, (message) -> hasNewLocalizedMap.set(true));
      messager.registerTopicListener(LiveMapAPI.RegionsAtFeet, (message) -> hasNewRegionsAtFeet.set(true));

      // FIXME fix the output topic name.
      combinedMapPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, outputTopic);
   }

   private void dispatchLocalizedMap(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapAPI.LocalizedMap, PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
   }

   private void dispatchRegionsAtFeet(Subscriber<PlanarRegionsListMessage> subscriber)
   {
      PlanarRegionsListMessage message = subscriber.takeNextData();
      messager.submitMessage(LiveMapAPI.RegionsAtFeet, PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
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

      boolean shouldUpdateMap = hasNewLocalizedMap.getAndSet(false) || hasNewRegionsAtFeet.getAndSet(false);

      if (shouldUpdateMap)
      {
         PlanarRegionsList localizedMap = mostRecentLocalizedMap.get();
         PlanarRegionsList regionsAtFeet = mostRecentRegionsAtFeet.get();

         PlanarRegionsList combinedMap = PlanarRegionSLAM.generateMergedMapByMergingAllPlanarRegionsMatches(localizedMap,
                                                                                                            regionsAtFeet,
                                                                                                            slamParameters.get(),
                                                                                                            null);

         messager.submitMessage(LiveMapAPI.CombinedLiveMap, combinedMap);
         combinedMapPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedMap));
      }
   }
}
