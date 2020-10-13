package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.atlas.sensors.AtlasSLAMBasedREAStandaloneLauncher;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.*;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.File;
import java.nio.file.Paths;
import java.util.List;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;

public class AtlasPerceptionSimulation
{
   private final boolean runRealsenseSLAM;
   private final boolean runLidarREA;
   private PausablePeriodicThread multisenseLidarPublisher;
   private PausablePeriodicThread multisenseRegionsPublisher;
   private PausablePeriodicThread realsenseRegionsPublisher;
   private AtlasSLAMBasedREAStandaloneLauncher realsenseSLAMFramework;
   private PausablePeriodicThread realsensePointCloudPublisher;
   private LIDARBasedREAModule lidarREA;

   public AtlasPerceptionSimulation(CommunicationMode communicationMode,
                                    PlanarRegionsList map,
                                    boolean runRealsenseSLAM,
                                    boolean spawnUIs,
                                    boolean simulateMultisenseLidar,
                                    boolean runLidarREA,
                                    DRCRobotModel robotModel)
   {
      this.runRealsenseSLAM = runRealsenseSLAM;
      this.runLidarREA = runLidarREA;
      ROS2Node ros2Node = ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), "perception");

      // might be a weird delay with threads at 0.5 hz depending on each other
      double period = 1.0;

      if (simulateMultisenseLidar)
      {
         MultisenseLidarSimulator multisenseLidar = new MultisenseLidarSimulator(robotModel, ros2Node, map);
         IHMCROS2Publisher<LidarScanMessage> publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
         multisenseLidar.addLidarScanListener(scan -> publisher.publish(PointCloudMessageTools.toLidarScanMessage(scan, multisenseLidar.getSensorPose())));
      }
      if (runLidarREA)
      {
         ExceptionTools.handle(() ->
         {
            REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(
                  ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), ROS2Tools.REA_NODE_NAME),
                  outputTopic,
                  lidarOutputTopic,
                  stereoOutputTopic,
                  depthOutputTopic
            );
            File reaConfigurationFile = Paths.get(System.getProperty("user.home")).resolve(".ihmc/REAModuleConfiguration.txt").toFile();
            lidarREA = LIDARBasedREAModule.createRemoteModule(new FilePropertyHelper(reaConfigurationFile), networkProvider);
            lidarREA.start();
         }, DefaultExceptionHandler.PRINT_STACKTRACE);
      }
      else
      {
         MultisenseHeadStereoSimulator multisenseStereo = new MultisenseHeadStereoSimulator(map, robotModel, ros2Node);
         IHMCROS2Publisher<PlanarRegionsListMessage> publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.LIDAR_REA_REGIONS);
         multisenseRegionsPublisher = new PausablePeriodicThread("MultisenseREARegionsPublisher", period,
            () -> publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(multisenseStereo.computeRegions())));
         multisenseRegionsPublisher.start();
      }

      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(map, robotModel, ros2Node);
      if (runRealsenseSLAM)
      {
         IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.D435_POINT_CLOUD);
         realsensePointCloudPublisher = new PausablePeriodicThread("RealsensePointCloudPublisher", period,
            () ->
            {
               List<Point3DReadOnly> pointCloud = realsense.getPointCloud();
               if (!pointCloud.isEmpty())
                  publisher.publish(PointCloudMessageTools.toStereoVisionPointCloudMessage(pointCloud, realsense.getSensorPose()));
            });
         realsensePointCloudPublisher.start();
         realsenseSLAMFramework = new AtlasSLAMBasedREAStandaloneLauncher(spawnUIs, communicationMode.getPubSubImplementation());
      }
      else
      {
         PlanarRegionSLAMMapper realsenseSLAM = new PlanarRegionSLAMMapper();
         IHMCROS2Publisher<PlanarRegionsListMessage> publisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS);
         realsenseRegionsPublisher = new PausablePeriodicThread("RealsenseSLAMPublisher", period,
            () -> publisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(realsenseSLAM.update(realsense.computeRegions()))));
         realsenseRegionsPublisher.start();
      }
   }

   public void destroy()
   {
      LogTools.info("Shutting down...");
      if (runLidarREA)
      {
         lidarREA.stop();
      }
      else
      {
         multisenseRegionsPublisher.stop();
      }
      if (runRealsenseSLAM)
      {
         realsensePointCloudPublisher.stop();
         realsenseSLAMFramework.stop();
      }
      else
      {
         realsenseRegionsPublisher.stop();
      }
   }
}
