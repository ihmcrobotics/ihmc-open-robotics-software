package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.stage.Stage;
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
                                                  () -> new SLAMPerceptionSuiteElement(this::createSLAMModuleInternal,
                                                                                       SLAMBasedEnvironmentAwarenessUI::creatIntraprocessUI),
                                                  messager,
                                                  PerceptionSuiteAPI.RunRealSenseSLAM,
                                                  PerceptionSuiteAPI.RunRealSenseSLAMUI,
                                                  PerceptionSuiteAPI.GUIRunRealSenseSLAM,
                                                  PerceptionSuiteAPI.GUIRunRealSenseSLAMUI);
      realsenseREAModule = new PerceptionSuiteComponent<>("RealSense REA",
                                                          () -> new REAPerceptionSuiteElement(this::createRealSenseREAModule,
                                                                                              stage -> LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(
                                                                                                    stage,
                                                                                                    NetworkPorts.REA_MODULE2_UI_PORT)),
                                                          messager,
                                                          PerceptionSuiteAPI.RunRealSenseREA,
                                                          PerceptionSuiteAPI.RunRealSenseREAUI,
                                                          PerceptionSuiteAPI.GUIRunRealSenseREA,
                                                          PerceptionSuiteAPI.GUIRunRealSenseREAUI);
      lidarREAModule = new PerceptionSuiteComponent<>("Lidar REA",
                                                      () -> new REAPerceptionSuiteElement(() -> LIDARBasedREAModule.createIntraprocessModule(
                                                            LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME,
                                                            ros2Node), LIDARBasedEnvironmentAwarenessUI::creatIntraprocessUI),
                                                      messager,
                                                      PerceptionSuiteAPI.RunLidarREA,
                                                      PerceptionSuiteAPI.RunLidarREAUI,
                                                      PerceptionSuiteAPI.GUIRunLidarREA,
                                                      PerceptionSuiteAPI.GUIRunLidarREAUI);
      segmentationModule = new PerceptionSuiteComponent<>("Segmentation",
                                                          () -> new SegmentationPerceptionSuiteElement(this::createSegmentationModule,
                                                                                                       PlanarSegmentationUI::createIntraprocessUI),
                                                          messager,
                                                          PerceptionSuiteAPI.RunMapSegmentation,
                                                          PerceptionSuiteAPI.RunMapSegmentationUI,
                                                          PerceptionSuiteAPI.GUIRunMapSegmentation,
                                                          PerceptionSuiteAPI.GUIRunMapSegmentationUI);
      liveMapModule = new PerceptionSuiteComponent<>("LiveMap",
                                                     () -> new LiveMapPerceptionSuiteElement(() -> LiveMapModule.createIntraprocess(ros2Node),
                                                                                             LiveMapUI::createIntraprocessUI),
                                                     messager,
                                                     PerceptionSuiteAPI.RunLiveMap,
                                                     PerceptionSuiteAPI.RunLiveMapUI,
                                                     PerceptionSuiteAPI.GUIRunLiveMap,
                                                     PerceptionSuiteAPI.GUIRunLiveMapUI);

      slamModule.attachDependentModule(segmentationModule);
   }

   protected SLAMModule createSLAMModule() throws Exception
   {
      return SLAMModule.createIntraprocessModule(ros2Node);
   }

   private SLAMModule createSLAMModuleInternal() throws Exception
   {
      SLAMModule slamModule = createSLAMModule();
      if (segmentationModule.getElement() != null)
         slamModule.attachOcTreeConsumer(segmentationModule.getElement().getPerceptionModule());

      return slamModule;
   }

   private PlanarSegmentationModule createSegmentationModule() throws Exception
   {
      PlanarSegmentationModule segmentationModule = PlanarSegmentationModule.createIntraprocessModule(SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME, ros2Node);
      if (slamModule.getElement() != null)
         slamModule.getElement().getPerceptionModule().attachOcTreeConsumer(segmentationModule);

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

      ros2Node.destroy();
   }

   public static PerceptionSuite createIntraprocess(Messager messager) throws Exception
   {
      return new PerceptionSuite(messager);
   }
}
