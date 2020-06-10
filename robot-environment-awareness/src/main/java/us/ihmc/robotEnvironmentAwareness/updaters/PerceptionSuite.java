package us.ihmc.robotEnvironmentAwareness.updaters;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class PerceptionSuite
{
   private static final int THREAD_PERIOD_MILLISECONDS = 200;

   protected static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private SLAMModule realSenseSLAMModule;
   private LIDARBasedREAModule lidarREAModule;
   private PlanarSegmentationModule segmentationModule;

   private Stage lidarREAStage;
   private LIDARBasedEnvironmentAwarenessUI lidarREAModuleUI;

   protected final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
   // TODO module for combining rea and segmentation

   private final Messager messager;

   public PerceptionSuite(Messager messager)
   {
      this.messager = messager;

      messager.registerTopicListener(PerceptionSuiteAPI.RunRealSenseSLAM,
                                     run -> runnable(run, this::startRealSenseSLAM, this::stopRealSenseSLAM, "RealSense SLAM"));
      messager.registerTopicListener(PerceptionSuiteAPI.RunMapSegmentation,
                                     run -> runnable(run, this::startMapSegmentation, this::stopMapSegmentation, "Map Segmentation"));
      messager.registerTopicListener(PerceptionSuiteAPI.RunLidarREA, run -> runnable(run, this::startLidarREA, this::stopLidarREA, "Lidar REA"));

      messager.registerTopicListener(PerceptionSuiteAPI.RunLidarREAUI, run -> runnable(run, this::startLidarREAUI, this::stopLidarREAUI, "Lidar REA UI"));
   }

   protected SLAMModule createSLAMModule() throws Exception
   {
      return SLAMModule.createIntraprocessModule(ros2Node, MODULE_CONFIGURATION_FILE_NAME);
   }

   private interface Command
   {
      void run() throws Exception;
   }

   private void runnable(boolean run, Command start, Command stop, String name)
   {
      if (run)
      {
         try
         {
            start.run();
         }
         catch (Exception e)
         {
            LogTools.warn("Failed to start " + name + ": " + e.getMessage());
         }
      }
      else
      {
         try
         {
            stop.run();
         }
         catch (Exception e)
         {
            LogTools.warn("Failed to stop " + name + ": " + e.getMessage());
         }
      }
   }

   public void start() throws Exception
   {
      messager.startMessager();
   }

   public void stop()
   {
      stopRealSenseSLAM();
      stopLidarREA();
      stopLidarREAUI();
      stopMapSegmentation();

      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         LogTools.warn(e.getMessage());
      }
      ros2Node.destroy();
   }

   private void startRealSenseSLAM() throws Exception
   {
      if (realSenseSLAMModule == null)
      {
         realSenseSLAMModule = createSLAMModule();
         realSenseSLAMModule.start();
      }
      else
      {
         throw new RuntimeException("RealSense SLAM is already running.");
      }
   }

   private void stopRealSenseSLAM()
   {
      if (realSenseSLAMModule != null)
      {
         realSenseSLAMModule.stop();
         realSenseSLAMModule = null;
      }

      messager.submitMessage(PerceptionSuiteAPI.RunRealSenseSLAMUI, false);
      messager.submitMessage(PerceptionSuiteAPI.RunMapSegmentation, false);
   }

   private void startLidarREA() throws Exception
   {
      if (lidarREAModule == null)
      {
         lidarREAModule = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, ros2Node);
         lidarREAModule.start();
      }
      else
      {
         throw new RuntimeException("Lidar REA is already running.");
      }
   }

   private void stopLidarREA()
   {
      if (lidarREAModule != null)
      {
         lidarREAModule.stop();
         lidarREAModule = null;
      }

      messager.submitMessage(PerceptionSuiteAPI.RunLidarREAUI, false);
   }

   private void startLidarREAUI() throws Exception
   {
      if (lidarREAModule == null)
      {
         LogTools.info("Lidar REA must be running first.");
         messager.submitMessage(PerceptionSuiteAPI.RunLidarREAUI, false);
      }

      if (lidarREAModuleUI == null)
      {
         Platform.runLater(() ->
                           {
                              lidarREAStage = new Stage();
                              try
                              {
                                 lidarREAModuleUI = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(lidarREAStage);
                                 lidarREAModuleUI.show();
                              }
                              catch (Exception e)
                              {
                                 LogTools.warn(e.getMessage());
                              }
                              lidarREAStage.setOnCloseRequest(event ->
                                                              {
                                                                 messager.submitMessage(PerceptionSuiteAPI.RunLidarREAUI, false);
                                                                 stopLidarREAUI();
                                                              });
                           });
      }
      else
      {
         stopLidarREAUI();
         throw new RuntimeException("Lidar REA UI is already running.");
      }
   }

   private void stopLidarREAUI()
   {
      if (lidarREAModuleUI != null)
      {
         Platform.runLater(() ->
                           {
                              lidarREAStage.close();
                              lidarREAModuleUI.stop();
                              lidarREAStage = null;
                              lidarREAModuleUI = null;
                           });
      }
   }

   private void startMapSegmentation() throws Exception
   {
      if (realSenseSLAMModule == null)
         LogTools.warn("Cannot start the segmentation module without the SLAM module running.");

      if (segmentationModule == null)
      {
         segmentationModule = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, ros2Node);
         segmentationModule.start();

         realSenseSLAMModule.attachOcTreeConsumer(segmentationModule);
      }
   }

   private void stopMapSegmentation()
   {
      if (segmentationModule != null)
      {
         if (realSenseSLAMModule != null)
            realSenseSLAMModule.removeOcTreeConsumer(segmentationModule);
         segmentationModule.stop();
         segmentationModule = null;
      }

      messager.submitMessage(PerceptionSuiteAPI.RunMapSegmentationUI, false);
   }

   public static PerceptionSuite createIntraprocess() throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(PerceptionSuiteAPI.API,
                                                                NetworkPorts.PERCEPTION_SUITE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      return new PerceptionSuite(moduleMessager);
   }
}
