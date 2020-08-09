package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.*;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;
import us.ihmc.robotEnvironmentAwareness.ui.controller.*;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SegmentationMeshViewer;

import java.io.File;
import java.io.IOException;

public class PlanarSegmentationUI implements PerceptionUI
{
   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultPlanarSegmentationUIConfiguration.txt";

   private final BorderPane mainPane;
   private final REAUIMessager uiMessager;
   private final SegmentationMeshViewer meshViewer;

   @FXML
   private OcTreeEssentialsAnchorPaneController ocTreeEssentialsAnchorPaneController;
   @FXML
   private RegionSegmentationAnchorPaneController regionSegmentationAnchorPaneController;
   @FXML
   private BoundingBoxAnchorPaneController boundingBoxAnchorPaneController;
   @FXML
   private CustomRegionMergeAnchorPaneController customRegionMergeAnchorPaneController;
   @FXML
   private PolygonizerAnchorPaneController polygonizerAnchorPaneController;
   @FXML
   private SegmentationDataExporterAnchorPaneController segmentationDataExporterAnchorPaneController;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   private PlanarSegmentationUI(REAUIMessager uiMessager, Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      // Client
      this.uiMessager = uiMessager;
      if (!uiMessager.isInternalMessagerOpen())
         uiMessager.startMessager();

      meshViewer = new SegmentationMeshViewer(uiMessager);

      // No need to anything with it beside instantiating it.
      new PlanarRegionSegmentationDataExporter(uiMessager,
                                               SegmentationModuleAPI.PlanarRegionsSegmentationState,
                                               SegmentationModuleAPI.UISegmentationDataExporterDirectory,
                                               SegmentationModuleAPI.UISegmentationDataExportRequest);
      // No need to anything with it beside instantiating it.
      new PlanarRegionDataExporter(uiMessager,
                                   SegmentationModuleAPI.PlanarRegionsState,
                                   SegmentationModuleAPI.UIPlanarRegionDataExporterDirectory,
                                   SegmentationModuleAPI.UIPlanarRegionDataExportRequest); // No need to anything with it beside instantiating it.

      initializeControllers(uiMessager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(meshViewer.getRoot());

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, SegmentationModuleAPI.RequestEntireModuleState);
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
      uiMessager.submitMessageInternal(SegmentationModuleAPI.RequestEntireModuleState, true);
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

      segmentationDataExporterAnchorPaneController.setUiPlanarRegionDataExporterDirectoryTopic(SegmentationModuleAPI.UIPlanarRegionDataExporterDirectory);
      segmentationDataExporterAnchorPaneController.setUiSegmentationDataExporterDirectoryTopic(SegmentationModuleAPI.UISegmentationDataExporterDirectory);
      segmentationDataExporterAnchorPaneController.setUiSegmentationDataExportRequestTopic(SegmentationModuleAPI.UISegmentationDataExportRequest);
      segmentationDataExporterAnchorPaneController.setUiPlanarRegionDataExportRequestTopic(SegmentationModuleAPI.UIPlanarRegionDataExportRequest);
      segmentationDataExporterAnchorPaneController.setConfigurationFile(configurationFile);
      segmentationDataExporterAnchorPaneController.setMainWindow(primaryStage);
      segmentationDataExporterAnchorPaneController.attachREAMessager(uiMessager);
      segmentationDataExporterAnchorPaneController.bindControls();;

      ocTreeEssentialsAnchorPaneController.setConfigurationFile(configurationFile);
      ocTreeEssentialsAnchorPaneController.attachREAMessager(uiMessager);
      ocTreeEssentialsAnchorPaneController.bindControls();

      boundingBoxAnchorPaneController.setConfigurationFile(configurationFile);
      boundingBoxAnchorPaneController.attachREAMessager(uiMessager);
      boundingBoxAnchorPaneController.bindControls();

      regionSegmentationAnchorPaneController.setPlanarRegionsSegmentationEnableTopic(SegmentationModuleAPI.PlanarRegionsSegmentationEnable);
      regionSegmentationAnchorPaneController.setPlanarRegionsSegmentationClearTopic(SegmentationModuleAPI.PlanarRegionsSegmentationClear);
      regionSegmentationAnchorPaneController.setSaveRegionUpdaterConfigurationTopic(SegmentationModuleAPI.SaveUpdaterConfiguration);
      regionSegmentationAnchorPaneController.setPlanarRegionsSegmentationParametersTopic(SegmentationModuleAPI.PlanarRegionsSegmentationParameters);
      regionSegmentationAnchorPaneController.setConfigurationFile(configurationFile);
      regionSegmentationAnchorPaneController.attachREAMessager(uiMessager);
      regionSegmentationAnchorPaneController.bindControls();

      customRegionMergeAnchorPaneController.setCustomRegionsMergingEnableTopic(SegmentationModuleAPI.CustomRegionsMergingEnable);
      customRegionMergeAnchorPaneController.setCustomRegionsClearTopic(SegmentationModuleAPI.CustomRegionsClear);
      customRegionMergeAnchorPaneController.setSaveRegionUpdaterConfigurationTopic(SegmentationModuleAPI.SaveUpdaterConfiguration);
      customRegionMergeAnchorPaneController.setCustomRegionsMergingParametersTopic(SegmentationModuleAPI.CustomRegionsMergingParameters);
      customRegionMergeAnchorPaneController.setConfigurationFile(configurationFile);
      customRegionMergeAnchorPaneController.attachREAMessager(uiMessager);
      customRegionMergeAnchorPaneController.bindControls();

      polygonizerAnchorPaneController.setPlanarRegionsPolygonizerEnableTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerEnable);
      polygonizerAnchorPaneController.setPlanarRegionsPolygonizerClearTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerClear);
      polygonizerAnchorPaneController.setPlanarRegionsIntersectionEnableTopic(SegmentationModuleAPI.PlanarRegionsIntersectionEnable);
      polygonizerAnchorPaneController.setUiPlanarRegionHideNodesTopic(SegmentationModuleAPI.UIPlanarRegionHideNodes);
      polygonizerAnchorPaneController.setSaveRegionUpdaterConfigurationTopic(SegmentationModuleAPI.SaveUpdaterConfiguration);
      polygonizerAnchorPaneController.setPlanarRegionsConcaveHullParametersTopic(SegmentationModuleAPI.PlanarRegionsConcaveHullParameters);
      polygonizerAnchorPaneController.setPlanarRegionsPolygonizerParametersTopic(SegmentationModuleAPI.PlanarRegionsPolygonizerParameters);
      polygonizerAnchorPaneController.setPlanarRegionsIntersectionParametersTopic(SegmentationModuleAPI.PlanarRegionsIntersectionParameters);
      polygonizerAnchorPaneController.setConfigurationFile(configurationFile);
      polygonizerAnchorPaneController.attachREAMessager(uiMessager);
      polygonizerAnchorPaneController.bindControls();
   }

   public void show()
   {
      refreshModuleState();
      primaryStage.show();
   }

   public void hide()
   {
      primaryStage.hide();
   }

   public void stop()
   {
      try
      {
         uiConnectionHandler.stop();
         uiMessager.closeMessager();

         meshViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static PlanarSegmentationUI createIntraprocessUI(Messager messager, Stage primaryStage) throws Exception
   {
      REAUIMessager uiMessager = new REAUIMessager(messager);
      uiMessager.startMessager();
      return new PlanarSegmentationUI(uiMessager, primaryStage);
   }
}
