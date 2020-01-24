package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class SLAMAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSLAMButton;

   public SLAMAnchorPaneController()
   {

   }

   @Override
   public void bindControls()
   {

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
      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      Stage stage = new Stage();
      Scene mainScene = new Scene(mainPane, 600, 400);
      stage.setScene(mainScene);
      stage.show();
   }
}
