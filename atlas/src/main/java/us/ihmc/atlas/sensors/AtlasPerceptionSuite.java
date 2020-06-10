package us.ihmc.atlas.sensors;

import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.*;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.ros2.Ros2Node;

public class AtlasPerceptionSuite
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private AtlasSLAMModule realSenseSLAMModule;
   private LIDARBasedREAModule lidarREAModule;
   private PlanarSegmentationModule segmentationModule;

   private final DRCRobotModel robotModel;
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
   // TODO module for combining rea and segmentation

   private final Messager messager;

   public AtlasPerceptionSuite(Messager messager)
   {
      this.messager = messager;
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      messager.registerTopicListener(PerceptionSuiteAPI.RunRealSenseSLAM, run ->
      {
         if (run)
         {
            try
            {
               startRealSenseSLAM();
            }
            catch (Exception e)
            {
               LogTools.warn("Failed to start RealSense SLAM: " + e.getMessage());
            }
         }
         else
         {
            stopRealSenseSLAM();
         }
      });
      messager.registerTopicListener(PerceptionSuiteAPI.RunMapSegmentation, run ->
      {
         if (run)
         {
            try
            {
               startMapSegmentation();
            }
            catch (Exception e)
            {
               LogTools.warn("Failed to start Map Segmentation: " + e.getMessage());
            }
         }
         else
         {
            stopMapSegmentation();
         }
      });
      messager.registerTopicListener(PerceptionSuiteAPI.RunLidarREA, run ->
      {
         if (run)
         {
            try
            {
               startLidarREA();
            }
            catch (Exception e)
            {
               LogTools.warn("Failed to start Lidar REA: " + e.getMessage());
            }
         }
         else
         {
            stopLidarREA();
         }
      });
   }

   public void start()
   {
   }


   public void stop()
   {
      stopRealSenseSLAM();
      stopLidarREA();
      stopMapSegmentation();

      ros2Node.destroy();
   }

   private void startRealSenseSLAM() throws Exception
   {
      if (realSenseSLAMModule == null)
      {
         realSenseSLAMModule = AtlasSLAMModule.createIntraprocessModule(ros2Node, robotModel, MODULE_CONFIGURATION_FILE_NAME);
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

   public static AtlasPerceptionSuite createIntraprocess() throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(PerceptionSuiteAPI.API,
                                                                NetworkPorts.SLAM_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      return new AtlasPerceptionSuite(moduleMessager);
   }

}
