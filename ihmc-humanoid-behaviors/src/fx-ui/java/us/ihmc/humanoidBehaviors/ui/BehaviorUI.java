package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.ui.behaviors.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.editors.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.FXUIGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   // Editors
   public static SnappedPositionEditor SNAPPED_POSITION_EDITOR;

   private final PlanarRegionsGraphic planarRegionsGraphic;
   private final JavaFXRobotVisualizer robotVisualizer;

   @FXML private PatrolBehaviorUIController patrolBehaviorUIController;

   public BehaviorUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParameters plannerParameters, VisibilityGraphsParameters visibilityGraphsParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, RobotContactPointParameters<RobotSide> contactPointParameters, WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      patrolBehaviorUIController.init(messager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      {
         /** TODO: Replace with View3DFactory.addDefaultLighting() when javafx-toolkit 0.12.8+ is released */
         double ambientValue = 0.7;
         double pointValue = 0.2;
         double pointDistance = 1000.0;
         Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
         view3dFactory.addNodeToView(new AmbientLight(ambientColor));
         Color indoorColor = Color.color(pointValue, pointValue, pointValue);
         view3dFactory.addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);
      }
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      planarRegionsGraphic = new PlanarRegionsGraphic(messager);
      SNAPPED_POSITION_EDITOR = new SnappedPositionEditor(messager, subScene);

      view3dFactory.addNodeToView(planarRegionsGraphic.getRoot());
      view3dFactory.addNodeToView(patrolBehaviorUIController.getRoot());

      if(fullHumanoidRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXRobotVisualizer(fullHumanoidRobotModelFactory);
         patrolBehaviorUIController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
      }

      SNAPPED_POSITION_EDITOR.start();

      planarRegionsGraphic.start();
      patrolBehaviorUIController.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1200, 800);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      SNAPPED_POSITION_EDITOR.stop();

      planarRegionsGraphic.stop();
      patrolBehaviorUIController.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();
   }

   public static BehaviorUI createMessagerUI(Stage primaryStage, JavaFXMessager messager,
                                             FootstepPlannerParameters plannerParameters,
                                             VisibilityGraphsParameters visibilityGraphsParameters,
                                             FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                                             RobotContactPointParameters<RobotSide> contactPointParameters,
                                             WalkingControllerParameters walkingControllerParameters)
         throws Exception
   {
      return new BehaviorUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullHumanoidRobotModelFactory,
                                   contactPointParameters, walkingControllerParameters);
   }

   public static class API
   {
      private static final SimpleMessagerAPIFactory apiFactory = new SimpleMessagerAPIFactory(BehaviorUI.class);

      public static final Topic<FXUIEditor> ActiveEditor = apiFactory.createTopic("ActiveEditor", FXUIEditor.class);
      public static final Topic<PlanarRegionsList> PlanarRegionsList = apiFactory.createTopic("PlanarRegionsList", PlanarRegionsList.class);
      public static final Topic<FXUIGraphic> SelectedGraphic = apiFactory.createTopic("SelectedGraphic", FXUIGraphic.class);
      public static final Topic<FXUIStateMachine> ActiveStateMachine = apiFactory.createTopic("ActiveStateMachine", FXUIStateMachine.class);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
