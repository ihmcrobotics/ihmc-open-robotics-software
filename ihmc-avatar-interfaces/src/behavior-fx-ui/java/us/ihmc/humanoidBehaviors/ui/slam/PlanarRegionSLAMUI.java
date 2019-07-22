package us.ihmc.humanoidBehaviors.ui.slam;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.LocalParameterServer;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

public class PlanarRegionSLAMUI extends Application
{
   @FXML
   private PlanarRegionSLAMUITabController planarRegionSLAMUITabController;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "behavior_ui");

      LocalParameterServer parameterServer = new LocalParameterServer(getClass(), 16784);
      LabelGraphic.initializeYoVariables(parameterServer.getRegistry());
      parameterServer.start();

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      BorderPane mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

      planarRegionSLAMUITabController.init(primaryStage, ros2Node);

      view3dFactory.addNodeToView(planarRegionSLAMUITabController);

      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1350, 900);

      primaryStage.setScene(mainScene);
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
