package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.Function;

import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.FootstepMeshViewer;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.SLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.controller.*;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SensorFrameViewer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class SLAMBasedEnvironmentAwarenessUI implements PerceptionUI
{
   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultSLAMUIConfiguration.txt";

   private final BorderPane mainPane;

   private final SLAMMeshViewer ihmcSLAMViewer;
   private final REAUIMessager uiMessager;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> depthFrameViewer;
   private final SensorFrameViewer<StampedPosePacket> pelvisFrameViewer;
   private final FootstepMeshViewer footstepViewer;
   private boolean shuttingDown = false;
   private boolean closedExternally = false;

   @FXML
   private SLAMAnchorPaneController slamAnchorPaneController;
   @FXML
   private SurfaceElementICPPaneController surfaceElementICPPaneController;
   @FXML
   private SLAMDataManagerAnchorPaneController slamDataManagerAnchorPaneController;
   @FXML
   private BoundingBoxAnchorPaneController boundingBoxAnchorPaneController;
   @FXML
   private NormalEstimationAnchorPaneController normalEstimationAnchorPaneController;
   @FXML
   private FrameNormalEstimationAnchorPaneController frameNormalEstimationAnchorPaneController;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   private final StereoVisionPointCloudDataExporter stereoVisionPointCloudDataExporter;
   private final ROS2Node ros2Node;

   public SLAMBasedEnvironmentAwarenessUI(REAUIMessager uiMessager, Stage primaryStage, SideDependentList<List<Point2D>> defaultContactPoints) throws Exception
   {
      this(uiMessager, primaryStage, defaultContactPoints, true);
   }

   public SLAMBasedEnvironmentAwarenessUI(REAUIMessager uiMessager,
                                           Stage primaryStage,
                                           SideDependentList<List<Point2D>> defaultContactPoints,
                                           boolean startMessager) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.setBackgroundColor(Color.WHITE);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      // Client
      this.uiMessager = uiMessager;
      if (startMessager)
         uiMessager.startMessager();

      ihmcSLAMViewer = new SLAMMeshViewer(uiMessager);
      depthFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager,
                                                                              SLAMModuleAPI.DepthPointCloudState,
                                                                              SLAMModuleAPI.UISensorPoseHistoryFrames,
                                                                              SensorFrameViewer.createStereoVisionSensorFrameExtractor(),
                                                                              SLAMModuleAPI.SensorPoseHistoryClear);
      view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
      view3dFactory.addNodeToView(depthFrameViewer.getRoot());

      stereoVisionPointCloudDataExporter = new StereoVisionPointCloudDataExporter(uiMessager,
                                                                                  SLAMModuleAPI.DepthPointCloudState,
                                                                                  SLAMModuleAPI.UIRawDataExportDirectory,
                                                                                  SLAMModuleAPI.UIRawDataExportRequest);

      if (defaultContactPoints == null)
      {
         pelvisFrameViewer = null;
         footstepViewer = null;
      }
      else
      {
         pelvisFrameViewer = new SensorFrameViewer<StampedPosePacket>(uiMessager,
                                                                      SLAMModuleAPI.CustomizedFrameState,
                                                                      SLAMModuleAPI.UISensorPoseHistoryFrames,
                                                                      SensorFrameViewer.createStampedPosePacketSensorFrameExtractor(),
                                                                      SLAMModuleAPI.SensorPoseHistoryClear);
         Function<RobotSide, ConvexPolygon2D> contactPointsProvider = new Function<RobotSide, ConvexPolygon2D>()
         {
            @Override
            public ConvexPolygon2D apply(RobotSide robotSide)
            {
               ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
               for (int i = 0; i < defaultContactPoints.get(robotSide).size(); i++)
               {
                  defaultFoothold.addVertex(defaultContactPoints.get(robotSide).get(i));
               }

               defaultFoothold.update();
               return defaultFoothold;
            }
         };
         footstepViewer = new FootstepMeshViewer(uiMessager, contactPointsProvider);

         view3dFactory.addNodeToView(pelvisFrameViewer.getRoot());
         view3dFactory.addNodeToView(footstepViewer.getRoot());
      }

      initializeControllers(uiMessager);

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, SLAMModuleAPI.RequestEntireModuleState);
      uiConnectionHandler.start();

      uiMessager.notifyModuleMessagerStateListeners();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);

      ros2Node = ROS2Tools.createROS2Node(FAST_RTPS, "slam_ui");
      IHMCROS2Publisher<Empty> shutdownPublisher = ROS2Tools.createPublisher(ros2Node, SLAMModuleAPI.SHUTDOWN);
      new IHMCROS2Callback<>(ros2Node, SLAMModuleAPI.SHUTDOWN, message ->
      {
         if (!shuttingDown)
         {
            LogTools.info("Received SHUTDOWN. Shutting down...");
            closeAndStop();
         }
         else
            LogTools.info("Received SHUTDOWN. Already shutting down...");
      });
      primaryStage.setOnCloseRequest(event ->
      {
         shuttingDown = true;
         if (!closedExternally)
         {
            ThreadTools.startAThread(() ->
            {
               shutdownPublisher.publish(new Empty());
               stop();
            }, "SLAMModuleUIShutdown");
         }
      });
   }

   private void refreshModuleState()
   {
      uiMessager.submitStateRequestToModule(SLAMModuleAPI.RequestEntireModuleState);
   }

   private void initializeControllers(REAUIMessager uiMessager)
   {
      File configurationFile = new File(UI_CONFIGURATION_FILE_NAME);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      slamAnchorPaneController.attachREAMessager(uiMessager);
      slamAnchorPaneController.bindControls();

      slamDataManagerAnchorPaneController.attachREAMessager(uiMessager);
      slamDataManagerAnchorPaneController.setMainWindow(primaryStage);
      slamDataManagerAnchorPaneController.setConfigurationFile(configurationFile);
      slamDataManagerAnchorPaneController.bindControls();

      surfaceElementICPPaneController.attachREAMessager(uiMessager);
      surfaceElementICPPaneController.bindControls();

      boundingBoxAnchorPaneController.setBoundingBoxEnableTopic(SLAMModuleAPI.OcTreeBoundingBoxEnable);
      boundingBoxAnchorPaneController.setBoundingBoxShowTopic(SLAMModuleAPI.UIOcTreeBoundingBoxShow);
      boundingBoxAnchorPaneController.setSaveParameterConfigurationTopic(SLAMModuleAPI.SaveConfiguration);
      boundingBoxAnchorPaneController.setBoundingBoxParametersTopic(SLAMModuleAPI.OcTreeBoundingBoxParameters);
      boundingBoxAnchorPaneController.setConfigurationFile(configurationFile);
      boundingBoxAnchorPaneController.attachREAMessager(uiMessager);
      boundingBoxAnchorPaneController.bindControls();;

      normalEstimationAnchorPaneController.setNormalEstimationEnableTopic(SLAMModuleAPI.NormalEstimationEnable);
      normalEstimationAnchorPaneController.setNormalEstimationClearTopic(SLAMModuleAPI.NormalEstimationClear);
      normalEstimationAnchorPaneController.setSaveMainUpdaterConfigurationTopic(SLAMModuleAPI.SaveConfiguration);
      normalEstimationAnchorPaneController.setNormalEstimationParametersTopic(SLAMModuleAPI.NormalEstimationParameters);
      normalEstimationAnchorPaneController.setConfigurationFile(configurationFile);
      normalEstimationAnchorPaneController.attachREAMessager(uiMessager);
      normalEstimationAnchorPaneController.bindControls();

      frameNormalEstimationAnchorPaneController.setConfigurationFile(configurationFile);
      frameNormalEstimationAnchorPaneController.attachREAMessager(uiMessager);
      frameNormalEstimationAnchorPaneController.bindControls();
   }

   @Override
   public void show()
   {
      refreshModuleState();
      primaryStage.show();
   }

   public void closeAndStop()
   {
      closedExternally = true;
      Platform.runLater(() ->
      {
         uiConnectionHandler.stop();
         primaryStage.close();
      });
      stop();
   }

   @Override
   public void stop()
   {
      try
      {
         uiConnectionHandler.stop();
         uiMessager.closeMessager();

         ihmcSLAMViewer.stop();
         depthFrameViewer.stop();
         if (pelvisFrameViewer != null)
            pelvisFrameViewer.stop();
         if (footstepViewer != null)
            footstepViewer.stop();

         slamDataManagerAnchorPaneController.destroy();

         stereoVisionPointCloudDataExporter.shutdown();

         ros2Node.destroy();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static SLAMBasedEnvironmentAwarenessUI creatIntraprocessUI(Messager messager, Stage primaryStage) throws Exception
   {
      REAUIMessager uiMessager = new REAUIMessager(messager);
      return new SLAMBasedEnvironmentAwarenessUI(uiMessager, primaryStage, null);
   }

   public static SLAMBasedEnvironmentAwarenessUI creatIntraprocessUI(Messager messager, Stage primaryStage,
                                                                     SideDependentList<List<Point2D>> defaultContactPoints)
         throws Exception
   {
      REAUIMessager uiMessager = new REAUIMessager(messager);
      return new SLAMBasedEnvironmentAwarenessUI(uiMessager, primaryStage, defaultContactPoints);
   }
}
