package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.File;
import java.io.IOException;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;
import us.ihmc.robotEnvironmentAwareness.ui.controller.*;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.REAMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SensorFrameViewer;

public class LIDARBasedEnvironmentAwarenessUI implements PerceptionUI
{
   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAUIConfiguration.txt";

   private final BorderPane mainPane;

   private final REAUIMessager uiMessager;
   private final REAMeshViewer reaMeshViewer;
   private final SensorFrameViewer<LidarScanMessage> lidarFrameViewer;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> stereoFrameViewer;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> depthFrameViewer;
   private final SensorFrameViewer<StampedPosePacket> trackingFrameViewer;

   @FXML
   private PointCloudAnchorPaneController pointCloudAnchorPaneController;
   @FXML
   private OcTreeBasicsAnchorPaneController ocTreeBasicsAnchorPaneController;
   @FXML
   private SurfaceNormalFilterAnchorPaneController surfaceNormalFilterAnchorPaneController;
   @FXML
   private LIDARFilterAnchorPaneController lidarFilterAnchorPaneController;
   @FXML
   private NormalEstimationAnchorPaneController normalEstimationAnchorPaneController;
   @FXML
   private RegionSegmentationAnchorPaneController regionSegmentationAnchorPaneController;
   @FXML
   private CustomRegionMergeAnchorPaneController customRegionMergeAnchorPaneController;
   @FXML
   private PolygonizerAnchorPaneController polygonizerAnchorPaneController;
   @FXML
   private DataExporterAnchorPaneController dataExporterAnchorPaneController;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   private final StereoVisionPointCloudDataExporter stereoVisionPointCloudDataExporter;

   private LIDARBasedEnvironmentAwarenessUI(REAUIMessager uiMessager, Stage primaryStage) throws Exception
   {
      this(uiMessager, primaryStage, true);
   }

   public LIDARBasedEnvironmentAwarenessUI(REAUIMessager uiMessager, Stage primaryStage, boolean startMessager) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      // Client
      this.uiMessager = uiMessager;
      if (startMessager)
         uiMessager.startMessager();

      reaMeshViewer = new REAMeshViewer(uiMessager);
      lidarFrameViewer = new SensorFrameViewer<LidarScanMessage>(uiMessager,
                                                                 REAModuleAPI.LidarScanState,
                                                                 null,
                                                                 SensorFrameViewer.createLidarScanSensorFrameExtractor(),
                                                                 REAModuleAPI.UISensorPoseHistoryClear);
      stereoFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager,
                                                                               REAModuleAPI.StereoVisionPointCloudState,
                                                                               REAModuleAPI.UISensorPoseHistoryFrames,
                                                                               SensorFrameViewer.createStereoVisionSensorFrameExtractor(),
                                                                               REAModuleAPI.UISensorPoseHistoryClear);
      depthFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager,
                                                                              REAModuleAPI.DepthPointCloudState,
                                                                              REAModuleAPI.UISensorPoseHistoryFrames,
                                                                              SensorFrameViewer.createStereoVisionSensorFrameExtractor(),
                                                                              REAModuleAPI.UISensorPoseHistoryClear);
      trackingFrameViewer = new SensorFrameViewer<StampedPosePacket>(uiMessager,
                                                                     REAModuleAPI.TrackingCameraMessageState,
                                                                     REAModuleAPI.UISensorPoseHistoryFrames,
                                                                     SensorFrameViewer.createStampedPosePacketSensorFrameExtractor(),
                                                                     REAModuleAPI.UISensorPoseHistoryClear);
      new PlanarRegionSegmentationDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      new PlanarRegionDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      stereoVisionPointCloudDataExporter = new StereoVisionPointCloudDataExporter(uiMessager);

      initializeControllers(uiMessager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(reaMeshViewer.getRoot());
      view3dFactory.addNodeToView(lidarFrameViewer.getRoot());
      view3dFactory.addNodeToView(stereoFrameViewer.getRoot());
      view3dFactory.addNodeToView(depthFrameViewer.getRoot());
      view3dFactory.addNodeToView(trackingFrameViewer.getRoot());

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager);
      uiConnectionHandler.start();

      uiMessager.notifyModuleMessagerStateListeners();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      mainScene.setOnKeyPressed(event ->
      {
         if (event.getCode() == KeyCode.F5)
            refreshModuleState();
      });

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void refreshModuleState()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.RequestEntireModuleState);
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

      pointCloudAnchorPaneController.setConfigurationFile(configurationFile);
      pointCloudAnchorPaneController.attachREAMessager(uiMessager);
      pointCloudAnchorPaneController.bindControls();

      ocTreeBasicsAnchorPaneController.setConfigurationFile(configurationFile);
      ocTreeBasicsAnchorPaneController.attachREAMessager(uiMessager);
      ocTreeBasicsAnchorPaneController.bindControls();

      surfaceNormalFilterAnchorPaneController.setConfigurationFile(configurationFile);
      surfaceNormalFilterAnchorPaneController.attachREAMessager(uiMessager);
      surfaceNormalFilterAnchorPaneController.bindControls();

      lidarFilterAnchorPaneController.setConfigurationFile(configurationFile);
      lidarFilterAnchorPaneController.attachREAMessager(uiMessager);
      lidarFilterAnchorPaneController.bindControls();

      normalEstimationAnchorPaneController.setConfigurationFile(configurationFile);
      normalEstimationAnchorPaneController.attachREAMessager(uiMessager);
      normalEstimationAnchorPaneController.bindControls();

      regionSegmentationAnchorPaneController.setConfigurationFile(configurationFile);
      regionSegmentationAnchorPaneController.attachREAMessager(uiMessager);
      regionSegmentationAnchorPaneController.bindControls();

      customRegionMergeAnchorPaneController.setConfigurationFile(configurationFile);
      customRegionMergeAnchorPaneController.attachREAMessager(uiMessager);
      customRegionMergeAnchorPaneController.bindControls();

      polygonizerAnchorPaneController.setConfigurationFile(configurationFile);
      polygonizerAnchorPaneController.attachREAMessager(uiMessager);
      polygonizerAnchorPaneController.bindControls();

      dataExporterAnchorPaneController.setConfigurationFile(configurationFile);
      dataExporterAnchorPaneController.attachREAMessager(uiMessager);
      dataExporterAnchorPaneController.setMainWindow(primaryStage);
      dataExporterAnchorPaneController.bindControls();
   }

   public void show()
   {
      refreshModuleState();
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         uiConnectionHandler.stop();
         uiMessager.closeMessager();

         reaMeshViewer.stop();
         lidarFrameViewer.stop();
         stereoFrameViewer.stop();

         stereoVisionPointCloudDataExporter.shutdown();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static LIDARBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage) throws Exception
   {
      return creatIntraprocessUI(primaryStage, NetworkPorts.REA_MODULE_UI_PORT);
   }

   public static LIDARBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage, NetworkPorts networkPort) throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(REAModuleAPI.API,
                                                                networkPort,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new LIDARBasedEnvironmentAwarenessUI(uiMessager, primaryStage);
   }

   public static LIDARBasedEnvironmentAwarenessUI creatRemoteUI(Stage primaryStage, String host) throws Exception
   {
      Messager moduleMessager = KryoMessager.createTCPClient(REAModuleAPI.API,
                                                             host,
                                                             NetworkPorts.REA_MODULE_UI_PORT,
                                                             REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new LIDARBasedEnvironmentAwarenessUI(uiMessager, primaryStage);
   }
}
