package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.sensors.AtlasSLAMBasedREAStandaloneLauncher;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.MultisenseHeadStereoSimulator;
import us.ihmc.humanoidBehaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.tools.perception.PeriodicPointCloudPublisher;
import us.ihmc.humanoidBehaviors.tools.perception.RealsensePelvisSimulator;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class AtlasPerceptionSimulation
{
   private final PeriodicPlanarRegionPublisher multisenseRegionsPublisher;
   private final boolean runRealsenseSLAM;

   private PeriodicPlanarRegionPublisher realsenseRegionsPublisher;

   private AtlasSLAMBasedREAStandaloneLauncher realsenseSLAMFramework;
   private PeriodicPointCloudPublisher realsensePointCloudPublisher;

   public AtlasPerceptionSimulation(CommunicationMode communicationMode,
                                    PlanarRegionsList map,
                                    boolean runRealsenseSLAM,
                                    boolean spawnUIs,
                                    DRCRobotModel robotModel)
   {
      this.runRealsenseSLAM = runRealsenseSLAM;
      Ros2Node ros2Node = ROS2Tools.createRos2Node(communicationMode.getPubSubImplementation(), "perception");
      MultisenseHeadStereoSimulator multisense = new MultisenseHeadStereoSimulator(map, robotModel, ros2Node);
      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(map, robotModel, ros2Node);

      // might be a weird delay with threads at 0.5 hz depending on each other
      double period = 1.0;
      multisenseRegionsPublisher = new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, period, multisense);
      multisenseRegionsPublisher.start();

      if (runRealsenseSLAM)
      {
         realsensePointCloudPublisher = new PeriodicPointCloudPublisher(ros2Node, ROS2Tools.D435_POINT_CLOUD, period, realsense::getPointCloud, realsense::getSensorPose);
         realsensePointCloudPublisher.start();
         realsenseSLAMFramework = new AtlasSLAMBasedREAStandaloneLauncher(spawnUIs, communicationMode.getPubSubImplementation());
      }
      else
      {
         PlanarRegionSLAMMapper realsenseSLAM = new PlanarRegionSLAMMapper();
         realsenseRegionsPublisher = new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, period, () -> realsenseSLAM.update(realsense.get()));
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
