package us.ihmc.atlas.sensors;

import java.util.List;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PlanarSegmentationUI;
import us.ihmc.robotEnvironmentAwareness.ui.SLAMBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasSLAMBasedREAStandaloneLauncher
{
   private static boolean launchSegmentation = true;

   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultSegmentationModuleConfiguration.txt";
   private final boolean spawnUIs;
   private final DomainFactory.PubSubImplementation pubSubImplementation;

   private Ros2Node ros2Node;
   private Messager slamMessager;
   private Messager segmentationMessager;
   private SLAMBasedEnvironmentAwarenessUI ui;
   private AtlasSLAMModule module;

   private PlanarSegmentationUI planarSegmentationUI;
   private PlanarSegmentationModule segmentationModule;

   public AtlasSLAMBasedREAStandaloneLauncher(boolean spawnUIs, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this.spawnUIs = spawnUIs;
      this.pubSubImplementation = pubSubImplementation;

      Runnable setup = () -> ExceptionTools.handle(this::setup, DefaultExceptionHandler.PRINT_STACKTRACE);
      if (spawnUIs)
      {
         JavaFXApplicationCreator.createAJavaFXApplication();
         Platform.runLater(setup);
      }
      else
      {
         setup.run();
      }
   }

   public void setup() throws Exception
   {
      Stage primaryStage = null;
      if (spawnUIs)
      {
         primaryStage = new Stage();
      }

      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      RobotContactPointParameters<RobotSide> contactPointParameters = drcRobotModel.getContactPointParameters();
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }

      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, ROS2Tools.REA_NODE_NAME);

      slamMessager = spawnUIs ? new SharedMemoryJavaFXMessager(SLAMModuleAPI.API) : new SharedMemoryMessager(SLAMModuleAPI.API);
      slamMessager.startMessager();

      if (launchSegmentation)
      {
         segmentationMessager = spawnUIs ? new SharedMemoryJavaFXMessager(SegmentationModuleAPI.API) : new SharedMemoryMessager(SegmentationModuleAPI.API);
         segmentationMessager.startMessager();
      }

      if (spawnUIs)
      {
         ui = SLAMBasedEnvironmentAwarenessUI.creatIntraprocessUI(slamMessager, primaryStage, defaultContactPoints);
      }
      module = AtlasSLAMModule.createIntraprocessModule(ros2Node, drcRobotModel, slamMessager);

      Stage secondStage = null;
      if (launchSegmentation)
      {
         if (spawnUIs)
         {
            secondStage = new Stage();
            planarSegmentationUI = PlanarSegmentationUI.createIntraprocessUI(segmentationMessager, secondStage);
         }
         segmentationModule = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, ros2Node, segmentationMessager);
         module.attachOcTreeConsumer(segmentationModule);
      }

      if (spawnUIs)
      {
         primaryStage.setOnCloseRequest(event -> stop());
         if (secondStage != null)
            secondStage.setOnCloseRequest(event -> stop());

         ui.show();
         if (planarSegmentationUI != null)
            planarSegmentationUI.show();
      }

      module.start();
      if (segmentationModule != null)
         segmentationModule.start();
   }

   public void stop()
   {
      if (spawnUIs) ui.stop();
      module.stop();

      ExceptionTools.handle(() -> slamMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);

      if (launchSegmentation)
      {
         if (spawnUIs)
            planarSegmentationUI.stop();
         segmentationModule.stop();
         ExceptionTools.handle(() -> segmentationMessager.closeMessager(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }

      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new AtlasSLAMBasedREAStandaloneLauncher(true, DomainFactory.PubSubImplementation.FAST_RTPS);
   }
}
