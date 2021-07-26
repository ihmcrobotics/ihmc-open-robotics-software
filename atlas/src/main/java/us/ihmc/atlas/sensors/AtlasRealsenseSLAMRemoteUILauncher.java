package us.ihmc.atlas.sensors;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.processManagement.JavaProcessManager;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commons.exception.DefaultExceptionHandler.RUNTIME_EXCEPTION;
import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class AtlasRealsenseSLAMRemoteUILauncher
{
   private final PubSubImplementation pubSubImplementation = FAST_RTPS;

   private ROS2Node ros2Node;
   private Messager slamMessager;
   private Messager segmentationMessager;

   private SLAMBasedEnvironmentAwarenessUI ui;
   private PlanarSegmentationUI planarSegmentationUI;

   public AtlasRealsenseSLAMRemoteUILauncher()
   {
      Runnable setup = () -> ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE);

      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(setup);
   }

   public void setup() throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, ROS2Tools.REA_NODE_NAME);

      slamMessager = KryoMessager.createClient(SLAMModuleAPI.API,
                                               "poweredge",
                                               NetworkPorts.SLAM_MODULE_UI_PORT.getPort(),
                                               "KryoSLAMModuleMessager",
                                               5);
      ThreadTools.startAThread(() -> ExceptionTools.handle(() -> slamMessager.startMessager(), RUNTIME_EXCEPTION), "KryoMessagerAsyncConnectionThread");

      segmentationMessager = KryoMessager.createClient(SLAMModuleAPI.API,
                                                       "poweredge",
                                                       NetworkPorts.PLANAR_SEGMENTATION_UI_PORT.getPort(),
                                                       "KryoSegmentationModuleMessager",
                                                       5);
      ThreadTools.startAThread(() -> ExceptionTools.handle(() -> segmentationMessager.startMessager(), RUNTIME_EXCEPTION), "KryoMessagerAsyncConnectionThread");

      Stage primaryStage = new Stage();
      ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(slamMessager, primaryStage, defaultContactPoints);

      Stage secondStage = new Stage();
      planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);

      ui.show();
      planarSegmentationUI.show();

      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         LogTools.info("Received SHUTDOWN. Shutting down...");
         stop();
      });
   }

   public void stop()
   {
      ThreadTools.sleepSeconds(2.0);

      ExceptionTools.handle(() -> slamMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ExceptionTools.handle(() -> segmentationMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);

      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("AtlasSLAMBasedREA", AtlasRealsenseSLAMRemoteUILauncher::new);
      ArrayList<Process> processes = manager.spawnProcesses(AtlasRealsenseSLAMRemoteUILauncher.class, args);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "test_node");
      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
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
