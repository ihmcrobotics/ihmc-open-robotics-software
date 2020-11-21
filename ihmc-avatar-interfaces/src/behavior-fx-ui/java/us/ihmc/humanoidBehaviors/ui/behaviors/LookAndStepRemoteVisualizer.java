package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.ui.graphics.ConsoleScrollPane;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public class LookAndStepRemoteVisualizer
{
   private Stage primaryStage;

   public LookAndStepRemoteVisualizer(DRCRobotModel robotModel, ROS2NodeInterface ros2Node, Messager behaviorMessager)
   {
      LogTools.info("Launching...");
      JavaFXApplicationCreator.createAJavaFXApplication();

      Platform.runLater(() ->
      {
         AnchorPane mainAnchorPane = new AnchorPane();

         View3DFactory view3DFactory = View3DFactory.createSubscene();
         FocusBasedCameraMouseEventHandler camera = view3DFactory.addCameraController(0.05, 2000.0, true);
         double isoZoomOut = 10.0;
         camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
         view3DFactory.addWorldCoordinateSystem(0.3);
         view3DFactory.addDefaultLighting();
         SubScene subScene3D = view3DFactory.getSubScene();
         Pane view3DSubSceneWrappedInsidePane = view3DFactory.getSubSceneWrappedInsidePane();
         StackPane view3DStackPane = new StackPane(view3DSubSceneWrappedInsidePane);
         AnchorPane.setTopAnchor(view3DStackPane, 0.0);
         AnchorPane.setBottomAnchor(view3DStackPane, 0.0);
         AnchorPane.setLeftAnchor(view3DStackPane, 0.0);
         AnchorPane.setRightAnchor(view3DStackPane, 0.0);
         mainAnchorPane.getChildren().add(view3DStackPane);

         view3DFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));
         LookAndStepVisualizationGroup lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, behaviorMessager);
         lookAndStepVisualizationGroup.setEnabled(true);
         view3DFactory.addNodeToView(lookAndStepVisualizationGroup);

         ConsoleScrollPane consoleScrollPane = new ConsoleScrollPane(behaviorMessager, ros2Node);

         SplitPane mainSplitPane = new SplitPane();
         mainSplitPane.getItems().add(mainAnchorPane);
         mainSplitPane.getItems().add(consoleScrollPane);
         mainSplitPane.setDividerPositions(2.0 / 3.0);

         primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         Scene mainScene = new Scene(mainSplitPane, 1200, 800);

         primaryStage.setScene(mainScene);

         LogTools.info("Showing window");
         primaryStage.show();

         consoleScrollPane.setupAtEnd();
      });
   }

   public void close()
   {
      primaryStage.close();
   }
}
