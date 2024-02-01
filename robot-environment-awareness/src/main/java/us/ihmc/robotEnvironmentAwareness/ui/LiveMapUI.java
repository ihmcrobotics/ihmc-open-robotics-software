package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.*;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;
import us.ihmc.robotEnvironmentAwareness.ui.controller.LiveMapAnchorPaneController;
import us.ihmc.robotEnvironmentAwareness.ui.controller.PlanarRegionSLAMParametersUIController;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.LiveMapMeshViewer;

import java.io.File;
import java.io.IOException;

public class LiveMapUI implements PerceptionUI
{
   private static final String UI_CONFIGURATION_FILE_NAME = "./Configurations/defaultLiveMapUIConfiguration.txt";

   private final BorderPane mainPane;
   private final REAUIMessager uiMessager;
   private final LiveMapMeshViewer meshViewer;

   private final Stage primaryStage;

   private final UIConnectionHandler uiConnectionHandler;

   @FXML
   private LiveMapAnchorPaneController liveMapAnchorPaneController;
   @FXML
   private PlanarRegionSLAMParametersUIController slamParametersUIController;

   private LiveMapUI(REAUIMessager uiMessager, Stage primaryStage) throws Exception
   {
      this(uiMessager, primaryStage, null);
   }

   private LiveMapUI(REAUIMessager uiMessager, Stage primaryStage, String configurationFileProject) throws Exception
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

      meshViewer = new LiveMapMeshViewer(uiMessager);

      slamParametersUIController.setupParameters();
      initializeControllers(uiMessager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      view3dFactory.addNodeToView(meshViewer.getRoot());

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, LiveMapModuleAPI.RequestEntireModuleState);
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
      uiMessager.submitMessageInternal(LiveMapModuleAPI.RequestEntireModuleState, true);
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

      liveMapAnchorPaneController.setConfigurationFile(configurationFile);
      liveMapAnchorPaneController.attachREAMessager(uiMessager);
      liveMapAnchorPaneController.bindControls();

      slamParametersUIController.setConfigurationFile(configurationFile);
      slamParametersUIController.attachREAMessager(uiMessager);
      slamParametersUIController.bindControls();
   }

   public void show()
   {
      refreshModuleState();
      primaryStage.show();
      slamParametersUIController.onPrimaryStageLoaded();
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

   public static LiveMapUI createIntraprocessUI(Messager messager, Stage primaryStage) throws Exception
   {
      REAUIMessager uiMessager = new REAUIMessager(messager);
      uiMessager.startMessager();
      return new LiveMapUI(uiMessager, primaryStage);
   }
}
