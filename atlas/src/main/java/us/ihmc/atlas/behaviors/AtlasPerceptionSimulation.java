package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.sensors.AtlasSLAMBasedREAStandaloneLauncher;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.*;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;

public class AtlasPerceptionSimulation
{
   private final PeriodicPlanarRegionPublisher multisenseRegionsPublisher;
   private final boolean runRealsenseSLAM;
   private PeriodicPlanarRegionPublisher realsenseRegionsPublisher;
   private AtlasSLAMBasedREAStandaloneLauncher realsenseSLAMFramework;
   private PeriodicPointCloudPublisher realsensePointCloudPublisher;
   private final PeriodicPointCloudPublisher multisenseLidarPublisher;

   public AtlasPerceptionSimulation(CommunicationMode communicationMode,
                                    PlanarRegionsList map,
                                    boolean runRealsenseSLAM,
                                    boolean spawnUIs,
                                    DRCRobotModel robotModel)
   {
      this.runRealsenseSLAM = runRealsenseSLAM;
      ROS2Node ros2Node = ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), "perception");
      MultisenseLidarSimulator multisenseLidar = new MultisenseLidarSimulator(robotModel, ros2Node, map);
      MultisenseHeadStereoSimulator multisenseStereo = new MultisenseHeadStereoSimulator(map, robotModel, ros2Node);
      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(map, robotModel, ros2Node);

      // might be a weird delay with threads at 0.5 hz depending on each other
      double period = 1.0;
      multisenseRegionsPublisher = new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, period, multisenseStereo); // TODO use lidar
      multisenseRegionsPublisher.start();

      multisenseLidarPublisher = new PeriodicPointCloudPublisher(ros2Node,
                                                                 ROS2Tools.MULTISENSE_LIDAR_POINT_CLOUD,
                                                                 UnitConversions.hertzToSeconds(10.0),
                                                                 multisenseLidar::getPointCloud,
                                                                 multisenseLidar::getSensorPose);
      multisenseLidarPublisher.start();

      if (runRealsenseSLAM)
      {
         realsensePointCloudPublisher = new PeriodicPointCloudPublisher(ros2Node,
                                                                        ROS2Tools.D435_POINT_CLOUD,
                                                                        period,
                                                                        realsense::getPointCloud,
                                                                        realsense::getSensorPose);
         realsensePointCloudPublisher.start();
         realsenseSLAMFramework = new AtlasSLAMBasedREAStandaloneLauncher(spawnUIs, communicationMode.getPubSubImplementation());
      }
      else
      {
         PlanarRegionSLAMMapper realsenseSLAM = new PlanarRegionSLAMMapper();
         realsenseRegionsPublisher = new PeriodicPlanarRegionPublisher(ros2Node,
                                                                       ROS2Tools.REALSENSE_SLAM_REGIONS,
                                                                       period,
                                                                       () -> realsenseSLAM.update(realsense.get()));
         realsenseRegionsPublisher.start();
      }
   }

   public void destroy()
   {
      LogTools.info("Shutting down...");
      multisenseRegionsPublisher.stop();
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
