package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.ui.LiveMapUI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.LiveMapModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.WorkspacePathTools;

import java.nio.file.Path;
import java.nio.file.Paths;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class PerceptionSuite
{
   private static final String SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME = "defaultSegmentationModuleConfiguration.txt";
   private static final String LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME = "defaultREAModuleConfiguration.txt";
   private static final String REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME = "defaultRealSenseREAModuleConfiguration.txt";

//   private final Path slamConfigurationFilePath;
   private final Path segmentationConfigurationFilePath;
   private final Path realsenseREAConfigurationFilePath;

   private final PerceptionSuiteComponent<SLAMModule, SLAMBasedEnvironmentAwarenessUI> slamModule;
   private final PerceptionSuiteComponent<LIDARBasedREAModule, LIDARBasedEnvironmentAwarenessUI> realsenseREAModule;
   private final PerceptionSuiteComponent<LIDARBasedREAModule, LIDARBasedEnvironmentAwarenessUI> lidarREAModule;
   private final PerceptionSuiteComponent<PlanarSegmentationModule, PlanarSegmentationUI> segmentationModule;
   private final PerceptionSuiteComponent<LiveMapModule, LiveMapUI> liveMapModule;

   protected final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, PerceptionAPI.REA_NODE_NAME);

   private static final Path defaultRootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final String defaultDirectory = "/robot-environment-awareness/src/main/resources/";

   private final Messager messager;

   public PerceptionSuite(Messager messager)
   {
      this(messager,
           Paths.get(defaultRootPath.toString(), defaultDirectory + SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME),
           Paths.get(defaultRootPath.toString(), defaultDirectory + LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME),
           Paths.get(defaultRootPath.toString(), defaultDirectory + REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME),
           null);
   }

   public PerceptionSuite(Messager messager,
                          Path segmentationConfigurationFilePath,
                          Path lidarREAConfigurationFilePath,
                          Path realsenseREAConfigurationFilePath,
                          String liveMapProject)
   {
      this.messager = messager;
      this.segmentationConfigurationFilePath = segmentationConfigurationFilePath;
      this.realsenseREAConfigurationFilePath = realsenseREAConfigurationFilePath;

//      slamConfigurationFilePath = Paths.get(rootPath.toString(), directory + SLAM_CONFIGURATION_FILE_NAME);

      REANetworkProvider lidarREANetworkProvider = new LidarREANetworkProvider(ros2Node, outputTopic, lidarOutputTopic);
      REANetworkProvider realSenseREANetworkProvider = new RealSenseREANetworkProvider(ros2Node, stereoInputTopic, stereoOutputTopic);
      slamModule = new PerceptionSuiteComponent<>("RealSense SLAM",
                                                  () -> new SLAMPerceptionSuiteElement(this::createSLAMModuleInternal,
                                                                                       SLAMBasedEnvironmentAwarenessUI::creatIntraprocessUI),
                                                  messager,
                                                  PerceptionSuiteAPI.RunRealSenseSLAM,
                                                  PerceptionSuiteAPI.RunRealSenseSLAMUI,
                                                  PerceptionSuiteAPI.GUIRunRealSenseSLAM,
                                                  PerceptionSuiteAPI.GUIRunRealSenseSLAMUI);
      realsenseREAModule = new PerceptionSuiteComponent<>("RealSense REA",
                                                          () -> new REAPerceptionSuiteElement(m -> createRealSenseREAModule(realSenseREANetworkProvider),
                                                                                              (m, s) -> LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(s,
                                                                                                                                                             NetworkPorts.REA_MODULE2_UI_PORT)),
                                                          messager,
                                                          PerceptionSuiteAPI.RunRealSenseREA,
                                                          PerceptionSuiteAPI.RunRealSenseREAUI,
                                                          PerceptionSuiteAPI.GUIRunRealSenseREA,
                                                          PerceptionSuiteAPI.GUIRunRealSenseREAUI);
      lidarREAModule = new PerceptionSuiteComponent<>("Lidar REA",
                                                      () -> new REAPerceptionSuiteElement(m -> LIDARBasedREAModule.createIntraprocessModule(
                                                            new FilePropertyHelper(lidarREAConfigurationFilePath.toFile()),
                                                            lidarREANetworkProvider), (m, s) -> LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(s)),
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
                                                     () -> new LiveMapPerceptionSuiteElement(m -> LiveMapModule.createIntraprocess(ros2Node, m, liveMapProject),
                                                                                             LiveMapUI::createIntraprocessUI),
                                                     messager,
                                                     PerceptionSuiteAPI.RunLiveMap,
                                                     PerceptionSuiteAPI.RunLiveMapUI,
                                                     PerceptionSuiteAPI.GUIRunLiveMap,
                                                     PerceptionSuiteAPI.GUIRunLiveMapUI);

      slamModule.attachDependentModule(segmentationModule);
   }

   protected SLAMModule createSLAMModule(Messager messager)
   {
      return SLAMModule.createIntraprocessModule(ros2Node, messager);
   }

   private SLAMModule createSLAMModuleInternal(Messager messager)
   {
      SLAMModule slamModule = createSLAMModule(messager);
      if (segmentationModule.getElement() != null)
         slamModule.attachOcTreeConsumer(segmentationModule.getElement().getPerceptionModule());

      return slamModule;
   }

   private PlanarSegmentationModule createSegmentationModule(Messager messager) throws Exception
   {
      PlanarSegmentationModule segmentationModule = PlanarSegmentationModule.createIntraprocessModule(segmentationConfigurationFilePath.toFile(),
                                                                                                      ros2Node,
                                                                                                      messager);
      if (slamModule.getElement() != null)
         slamModule.getElement().getPerceptionModule().attachOcTreeConsumer(segmentationModule);

      return segmentationModule;
   }

   private LIDARBasedREAModule createRealSenseREAModule(REANetworkProvider networkProvider) throws Exception
   {
      LIDARBasedREAModule module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(realsenseREAConfigurationFilePath.toFile()),
                                                                                networkProvider,
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

   public static PerceptionSuite createIntraprocess(Messager messager)
   {
      return new PerceptionSuite(messager);
   }
}
