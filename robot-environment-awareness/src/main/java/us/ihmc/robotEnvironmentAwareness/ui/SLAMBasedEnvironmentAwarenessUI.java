package us.ihmc.robotEnvironmentAwareness.ui;

import java.io.IOException;
import java.util.List;

import controller_msgs.msg.dds.StampedPosePacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.FootstepMeshViewer;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.SLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.controller.SLAMAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.SLAMDataManagerAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SensorFrameViewer;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SLAMBasedEnvironmentAwarenessUI
{
   private final BorderPane mainPane;

   private final SLAMMeshViewer ihmcSLAMViewer;
   private final REAUIMessager uiMessager;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> depthFrameViewer;
   private final SensorFrameViewer<StampedPosePacket> pelvisFrameViewer;
   private final FootstepMeshViewer footstepViewer;

   @FXML
   private SLAMAnchorPaneController slamAnchorPaneController;
   @FXML
   private SLAMDataManagerAnchorPaneController slamDataManagerAnchorPaneController;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   private final StereoVisionPointCloudDataExporter stereoVisionPointCloudDataExporter;

   private SLAMBasedEnvironmentAwarenessUI(REAUIMessager uiMessager, Stage primaryStage, SideDependentList<List<Point2D>> defaultContactPoints) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      // Client
      this.uiMessager = uiMessager;
      uiMessager.startMessager();

      ihmcSLAMViewer = new SLAMMeshViewer(uiMessager);
      depthFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager,
                                                                              SLAMModuleAPI.DepthPointCloudState,
                                                                              SLAMModuleAPI.UISensorPoseHistoryFrames,
                                                                              SensorFrameViewer.createStereoVisionSensorFrameExtractor(),
                                                                              SLAMModuleAPI.SensorPoseHistoryClear);

      view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
      view3dFactory.addNodeToView(depthFrameViewer.getRoot());

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
         footstepViewer = new FootstepMeshViewer(uiMessager);
         footstepViewer.setDefaultContactPoints(defaultContactPoints);

         view3dFactory.addNodeToView(pelvisFrameViewer.getRoot());
         view3dFactory.addNodeToView(footstepViewer.getRoot());
      }

      new PlanarRegionSegmentationDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      new PlanarRegionDataExporter(uiMessager); // No need to anything with it beside instantiating it.
      stereoVisionPointCloudDataExporter = new StereoVisionPointCloudDataExporter(uiMessager,
                                                                                  SLAMModuleAPI.DepthPointCloudState,
                                                                                  SLAMModuleAPI.UIRawDataExportDirectory,
                                                                                  SLAMModuleAPI.UIRawDataExportRequest);

      initializeControllers(uiMessager);

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, SLAMModuleAPI.RequestEntireModuleState);
      uiConnectionHandler.start();

      uiMessager.notifyModuleMessagerStateListeners();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void refreshModuleState()
   {
      uiMessager.submitStateRequestToModule(SLAMModuleAPI.RequestEntireModuleState);
   }

   private void initializeControllers(REAUIMessager uiMessager)
   {
      slamAnchorPaneController.attachREAMessager(uiMessager);
      slamAnchorPaneController.bindControls();

      slamDataManagerAnchorPaneController.attachREAMessager(uiMessager);
      slamDataManagerAnchorPaneController.bindControls();
   }

   public void show() throws IOException
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

         ihmcSLAMViewer.stop();
         depthFrameViewer.stop();
         if (pelvisFrameViewer != null)
            pelvisFrameViewer.stop();
         if (footstepViewer != null)
            footstepViewer.stop();

         stereoVisionPointCloudDataExporter.shutdown();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static SLAMBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage) throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(SLAMModuleAPI.API,
                                                                NetworkPorts.SLAM_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new SLAMBasedEnvironmentAwarenessUI(uiMessager, primaryStage, null);
   }

   public static SLAMBasedEnvironmentAwarenessUI creatIntraprocessUI(Stage primaryStage, SideDependentList<List<Point2D>> defaultContactPoints) throws Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(SLAMModuleAPI.API,
                                                                NetworkPorts.SLAM_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new SLAMBasedEnvironmentAwarenessUI(uiMessager, primaryStage, defaultContactPoints);
   }
}
