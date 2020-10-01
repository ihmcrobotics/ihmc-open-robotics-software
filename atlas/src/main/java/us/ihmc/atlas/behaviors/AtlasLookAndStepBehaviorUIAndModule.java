package us.ihmc.atlas.behaviors;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.JavaProcessManager;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class AtlasLookAndStepBehaviorUIAndModule
{
   public static final boolean SHOW_REALSENSE_SLAM_UIS = Boolean.parseBoolean(System.getProperty("show.realsense.slam.uis", "false"));

   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("AtlasBehaviorUIAndModule", () -> new AtlasBehaviorUIAndModule(BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION)));
//      manager.runOrRegister("RealsenseSLAM", () -> new AtlasSLAMBasedREAStandaloneLauncher(SHOW_REALSENSE_SLAM_UIS, PubSubImplementation.FAST_RTPS));
//      manager.runOrRegister("RealsenseREA", () -> new AtlasRealsenseBasedREAStandaloneLauncher(false));
//      manager.runOrRegister("LiveMap", () -> new LiveMapStandaloneLauncher(false, PubSubImplementation.FAST_RTPS));
//      manager.runOrRegister("LidarREA", () -> new LidarBasedREAStandaloneLauncher());
//      manager.runOrRegister("LidarREA", () -> new RemoteLidarBasedREAModuleLauncher());
      manager.runOrRegister("LidarREA", () ->
      {
         ExceptionTools.handle(() ->
         {
            REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic,
                                                                                          lidarOutputTopic,
                                                                                          stereoOutputTopic,
                                                                                          depthOutputTopic);
            File reaConfigurationFile = Paths.get(System.getProperty("user.home")).resolve(".ihmc/REAModuleConfiguration.txt").toFile();
            LIDARBasedREAModule module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(reaConfigurationFile),
                                                                                      networkProvider);
            module.start();
            ROS2Node ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "lidar_rea");
            new IHMCROS2Callback<>(ros2Node, BehaviorModule.API.SHUTDOWN, message -> module.stop());
         }, DefaultExceptionHandler.PRINT_STACKTRACE);
      });

      ArrayList<Process> processes = manager.spawnProcesses(AtlasLookAndStepBehaviorUIAndModule.class, args);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "test_node");
      new IHMCROS2Callback<>(ros2Node, BehaviorModule.API.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");

         ThreadTools.startAsDaemon(() ->
                                   {
                                      ThreadTools.sleepSeconds(2.0);
                                      for (Process process : processes)
                                      {
                                         LogTools.info("Destoying process  forcibly");
                                         process.destroyForcibly();
                                      }
                                      ros2Node.destroy();
                                   }, "DestroyThread");
      });
   }
}
