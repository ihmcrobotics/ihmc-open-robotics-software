package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.GPUModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionUI;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.PlanarRegionViewer;

public class GPUBasedEnvironmentAwarenessUI implements PerceptionUI
{

   private final BorderPane mainPane;
   private final Stage primaryStage;
   private final REAUIMessager uiMessager;
   private final UIConnectionHandler uiConnectionHandler;
   private final PlanarRegionViewer planarRegionViewer;

   public GPUBasedEnvironmentAwarenessUI(Messager messager, Stage primaryStage) throws Exception
   {

      uiMessager = new REAUIMessager(messager);
      uiMessager.startMessager();

      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      if (!uiMessager.isInternalMessagerOpen())
         uiMessager.startMessager();

      planarRegionViewer = new PlanarRegionViewer(messager, GPUModuleAPI.PlanarRegionData, GPUModuleAPI.ShowPlanarRegions);
      planarRegionViewer.start();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      uiConnectionHandler = new UIConnectionHandler(primaryStage, uiMessager, GPUModuleAPI.RequestEntireModuleState);
      uiConnectionHandler.start();

      uiMessager.notifyModuleMessagerStateListeners();

      primaryStage.setTitle(getClass().getSimpleName());
      //        primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public static GPUBasedEnvironmentAwarenessUI createIntraprocessUI(Messager messager, Stage primaryStage) throws Exception
   {
      return new GPUBasedEnvironmentAwarenessUI(messager, primaryStage);
   }

   @Override
   public void show()
   {
      primaryStage.show();
   }

   @Override
   public void stop()
   {

   }
}
