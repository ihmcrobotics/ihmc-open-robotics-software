package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.sensors.AtlasRealsenseBasedREAStandaloneLauncher;
import us.ihmc.atlas.sensors.AtlasSLAMBasedREAStandaloneLauncher;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.LiveMapStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.tools.processManagement.JavaProcessManager;

import java.io.File;
import java.nio.file.Paths;

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
         }, DefaultExceptionHandler.PRINT_STACKTRACE);
      });
      manager.spawnProcesses(AtlasLookAndStepBehaviorUIAndModule.class, args);
   }
}
