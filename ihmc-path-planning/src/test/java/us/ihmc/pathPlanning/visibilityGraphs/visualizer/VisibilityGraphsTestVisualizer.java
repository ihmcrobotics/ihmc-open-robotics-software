package us.ihmc.pathPlanning.visibilityGraphs.visualizer;

import java.io.IOException;
import java.text.DecimalFormat;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.BodyPathMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.ClusterMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.NavigableRegionInnerVizMapMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityGraphStartGoalViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VizGraphsPlanarRegionViewer;

public class VisibilityGraphsTestVisualizer
{
   private final BorderPane mainPane;
   private final Stage primaryStage;
   private final SimpleUIMessager messager;
   private final VizGraphsPlanarRegionViewer planarRegionViewer;
   private final VisibilityGraphStartGoalViewer startGoalViewer;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final NavigableRegionInnerVizMapMeshViewer navigableRegionInnerVizMapMeshViewer;
   private final ClusterMeshViewer clusterMeshViewer;

   @FXML
   private TextField startTextFieldX, startTextFieldY, startTextFieldZ;
   @FXML
   private TextField goalTextFieldX, goalTextFieldY, goalTextFieldZ;

   public VisibilityGraphsTestVisualizer(Stage primaryStage, SimpleUIMessager messager) throws IOException
   {
      this.primaryStage = primaryStage;
      this.messager = messager;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      bindTextFields(messager);

      planarRegionViewer = new VizGraphsPlanarRegionViewer(messager);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      startGoalViewer = new VisibilityGraphStartGoalViewer(messager);
      view3dFactory.addNodeToView(startGoalViewer.getRoot());
      bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      navigableRegionInnerVizMapMeshViewer = new NavigableRegionInnerVizMapMeshViewer(messager);
      view3dFactory.addNodeToView(navigableRegionInnerVizMapMeshViewer.getRoot());
      clusterMeshViewer = new ClusterMeshViewer(messager);
      view3dFactory.addNodeToView(clusterMeshViewer.getRoot());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void bindTextFields(SimpleUIMessager messager)
   {
      DecimalFormat formatter = new DecimalFormat(" 0.000;-0.000");
      messager.registerTopicListener(UIVisibilityGraphsTopics.StartPosition, start -> startTextFieldX.setText(formatter.format(start.getX())));
      messager.registerTopicListener(UIVisibilityGraphsTopics.StartPosition, start -> startTextFieldY.setText(formatter.format(start.getY())));
      messager.registerTopicListener(UIVisibilityGraphsTopics.StartPosition, start -> startTextFieldZ.setText(formatter.format(start.getZ())));
      messager.registerTopicListener(UIVisibilityGraphsTopics.GoalPosition, goal -> goalTextFieldX.setText(formatter.format(goal.getX())));
      messager.registerTopicListener(UIVisibilityGraphsTopics.GoalPosition, goal -> goalTextFieldY.setText(formatter.format(goal.getY())));
      messager.registerTopicListener(UIVisibilityGraphsTopics.GoalPosition, goal -> goalTextFieldZ.setText(formatter.format(goal.getZ())));
   }

   public void show() throws IOException
   {
      primaryStage.show();
      planarRegionViewer.start();
      startGoalViewer.start();
      bodyPathMeshViewer.start();
      navigableRegionInnerVizMapMeshViewer.start();
      clusterMeshViewer.start();
   }

   public void stop()
   {
      messager.closeMessager();
      planarRegionViewer.stop();
      startGoalViewer.stop();
      bodyPathMeshViewer.stop();
      navigableRegionInnerVizMapMeshViewer.stop();
      clusterMeshViewer.stop();
   }
}
