package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.IhmcSLAMMeshViewer;
import us.ihmc.robotEnvironmentAwareness.ui.UIConnectionHandler;

public class SLAMAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSLAMButton;

   private IhmcSLAMMeshViewer ihmcSLAMViewer;
   
   private UIConnectionHandler uiConnectionHandler;

   public SLAMAnchorPaneController()
   {

   }

   @Override
   public void bindControls()
   {
      ihmcSLAMViewer = new IhmcSLAMMeshViewer(uiMessager);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMEnable, enableSLAMButton.selectedProperty());
   }

   public void openMap() throws IOException
   {
      System.out.println("open Map.");

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource("../SLAMVisualizerMainPane" + ".fxml"));

      BorderPane mainPane = loader.load();
      
      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addNodeToView(ihmcSLAMViewer.getRoot());
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      Stage stage = new Stage();
      Scene mainScene = new Scene(mainPane, 600, 400);
      stage.setScene(mainScene);
      stage.setOnCloseRequest(event -> stop());
      
      uiConnectionHandler = new UIConnectionHandler(stage, uiMessager);
      uiConnectionHandler.start();
      
      stage.show();

      uiMessager.submitMessageToModule(REAModuleAPI.RequestSLAMBuildMap, true);
   }
   
   public void stop()
   {
      try
      {
         uiConnectionHandler.stop();
         ihmcSLAMViewer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
