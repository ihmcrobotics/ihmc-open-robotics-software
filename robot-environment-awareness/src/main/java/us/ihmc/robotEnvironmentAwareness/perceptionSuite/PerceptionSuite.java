package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.ros2.Ros2Node;

public class PerceptionSuite
{
   private static final String SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultSegmentationModuleConfiguration.txt";
   private static final String LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";
   private static final String REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultRealSenseREAModuleConfiguration.txt";

   private PerceptionSuiteComponent<SLAMModule, SLAMBasedEnvironmentAwarenessUI> slamModule;
   private PerceptionSuiteComponent<LIDARBasedREAModule, LIDARBasedEnvironmentAwarenessUI> realsenseREAModule;
   private PerceptionSuiteComponent<LIDARBasedREAModule, LIDARBasedEnvironmentAwarenessUI> lidarREAModule;
   private PerceptionSuiteComponent<PlanarSegmentationModule, PlanarSegmentationUI> segmentationModule;
   private PerceptionSuiteComponent<LiveMapModule, LiveMapUI> liveMapModule;

   protected final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);

   private final Messager messager;

   public PerceptionSuite(Messager messager)
   {
      this.messager = messager;

      slamModule = new PerceptionSuiteComponent<>("RealSense SLAM",
                                                  this::createSLAMModuleInternal,
                                                  SLAMBasedEnvironmentAwarenessUI::creatIntraprocessUI,
                                                  messager,
                                                  PerceptionSuiteAPI.RunRealSenseSLAM,
                                                  PerceptionSuiteAPI.RunRealSenseSLAMUI);
      realsenseREAModule = new PerceptionSuiteComponent<>("RealSense REA",
                                                          this::createRealSenseREAModule,
                                                          stage -> LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(stage,
                                                                                                                        NetworkPorts.REA_MODULE2_UI_PORT),
                                                          messager,
                                                          PerceptionSuiteAPI.RunRealSenseREA,
                                                          PerceptionSuiteAPI.RunRealSenseREAUI);
      lidarREAModule = new PerceptionSuiteComponent<>("Lidar REA",
                                                      () -> LIDARBasedREAModule.createIntraprocessModule(LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME, ros2Node),
                                                      LIDARBasedEnvironmentAwarenessUI::creatIntraprocessUI,
                                                      messager,
                                                      PerceptionSuiteAPI.RunLidarREA,
                                                      PerceptionSuiteAPI.RunLidarREAUI);
      segmentationModule = new PerceptionSuiteComponent<>("Segmentation",
                                                          this::createSegmentationModule,
                                                          PlanarSegmentationUI::createIntraprocessUI,
                                                          messager,
                                                          PerceptionSuiteAPI.RunMapSegmentation,
                                                          PerceptionSuiteAPI.RunMapSegmentationUI);
      liveMapModule = new PerceptionSuiteComponent<>("LiveMap",
                                                     () -> LiveMapModule.createIntraprocess(ros2Node),
                                                     LiveMapUI::createIntraprocessUI,
                                                     messager,
                                                     PerceptionSuiteAPI.RunLiveMap,
                                                     PerceptionSuiteAPI.RunLiveMapUI);

      slamModule.attachDependentModule(segmentationModule);
   }

   protected SLAMModule createSLAMModule() throws Exception
   {
      return SLAMModule.createIntraprocessModule(ros2Node);
   }

   private SLAMModule createSLAMModuleInternal() throws Exception
   {
      SLAMModule slamModule = createSLAMModule();
      if (segmentationModule.getModule() != null)
         slamModule.attachOcTreeConsumer(segmentationModule.getModule());

      return slamModule;
   }

   private PlanarSegmentationModule createSegmentationModule() throws Exception
   {
      PlanarSegmentationModule segmentationModule = PlanarSegmentationModule.createIntraprocessModule(SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME, ros2Node);
      if (slamModule.getModule() != null)
         slamModule.getModule().attachOcTreeConsumer(segmentationModule);

      return segmentationModule;
   }

   private LIDARBasedREAModule createRealSenseREAModule() throws Exception
   {
      LIDARBasedREAModule module = LIDARBasedREAModule.createIntraprocessModule(REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME,
                                                                                ros2Node,
                                                                                NetworkPorts.REA_MODULE2_UI_PORT);
      module.setParametersForStereo();
      module.loadConfigurationsFromFile();
      return module;
   }

   public void start() throws Exception
   {
      messager.startMessager();
   }

   public void stop()
   {
      slamModule.stop();
      lidarREAModule.stop();
      realsenseREAModule.stop();
      segmentationModule.stop();
      liveMapModule.stop();

      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         String error = e.getMessage();
         LogTools.error(error);
         messager.submitMessage(PerceptionSuiteAPI.ErrorMessage, error);
      }
      ros2Node.destroy();
   }

   public static PerceptionSuite createIntraprocess() throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(PerceptionSuiteAPI.API,
                                                                NetworkPorts.PERCEPTION_SUITE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      return new PerceptionSuite(moduleMessager);
   }
}
