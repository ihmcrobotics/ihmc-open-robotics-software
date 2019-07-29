package us.ihmc.footstepPlanning.ui;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabledTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalPositionTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LowLevelGoalPositionTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanarRegionDataTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RobotConfigurationDataTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.SelectedRegionTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowPlanarRegionsTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartOrientationEditModeEnabledTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartPositionTopic;

import java.util.ArrayList;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SaveableFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.ui.components.NodeCheckerEditor;
import us.ihmc.footstepPlanning.ui.components.OccupancyMapRenderer;
import us.ihmc.footstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.footstepPlanning.ui.controllers.BodyCollisionCheckingUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepNodeCheckingUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerCostsUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerDataExporterAnchorPaneController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerMenuUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.MainTabController;
import us.ihmc.footstepPlanning.ui.controllers.UIRobotController;
import us.ihmc.footstepPlanning.ui.controllers.VisibilityGraphsParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.VisualizationController;
import us.ihmc.footstepPlanning.ui.viewers.BodyPathMeshViewer;
import us.ihmc.footstepPlanning.ui.viewers.FootstepPathMeshViewer;
import us.ihmc.footstepPlanning.ui.viewers.NodeCheckerRenderer;
import us.ihmc.footstepPlanning.ui.viewers.StartGoalOrientationViewer;
import us.ihmc.footstepPlanning.ui.viewers.StartGoalPositionViewer;
import us.ihmc.footstepPlanning.ui.viewers.VisibilityGraphsRenderer;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

/**
 * This class is the visualization element of the footstep planner. It also contains a graphical interface for
 * setting planner parameters to be used by the footstep planner itself.
 */
public class FootstepPlannerUI
{
   private static final boolean VERBOSE = true;

   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final NodeCheckerEditor nodeCheckerEditor;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final FootstepPathMeshViewer pathViewer;
   private final StartGoalOrientationEditor orientationEditor;
   private final NodeCheckerRenderer nodeCheckerRenderer;
   private final FootstepPlannerDataExporter dataExporter;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
   private final OccupancyMapRenderer graphRenderer;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final JavaFXRobotVisualizer walkingPreviewVisualizer;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;
   @FXML
   private FootstepNodeCheckingUIController footstepNodeCheckingUIController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private BodyCollisionCheckingUIController bodyCollisionCheckingUIController;
   @FXML
   private FootstepPlannerCostsUIController footstepPlannerCostsUIController;
   @FXML
   private FootstepPlannerDataExporterAnchorPaneController dataExporterAnchorPaneController;
   @FXML
   private MainTabController mainTabController;
   @FXML
   private UIRobotController uiRobotController;

   @FXML
   private VisualizationController visibilityGraphsUIController;

   public FootstepPlannerUI(Stage primaryStage, SaveableFootstepPlannerParameters plannerParameters, VisibilityGraphsParameters visibilityGraphsParameters) throws Exception
   {
      this(primaryStage, new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API), plannerParameters, visibilityGraphsParameters, null, null, null);
      messager.startMessager();
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this(primaryStage, messager, new DefaultFootstepPlannerParameters(), new DefaultVisibilityGraphParameters(), null, null, null);
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, SaveableFootstepPlannerParameters plannerParameters,
                            VisibilityGraphsParameters visibilityGraphsParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            RobotContactPointParameters<RobotSide> contactPointParameters, WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullHumanoidRobotModelFactory, null, contactPointParameters,
           walkingControllerParameters);
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, SaveableFootstepPlannerParameters plannerParameters,
                            VisibilityGraphsParameters visibilityGraphsParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            FullHumanoidRobotModelFactory previewModelFactory, RobotContactPointParameters<RobotSide> contactPointParameters,
                            WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      footstepPlannerCostsUIController.setPlannerParameters(plannerParameters);
      footstepPlannerParametersUIController.setPlannerParameters(plannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);
      bodyCollisionCheckingUIController.setPlannerParameters(plannerParameters);

      mainTabController.attachMessager(messager);
      footstepPlannerMenuUIController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      bodyCollisionCheckingUIController.attachMessager(messager);
      footstepPlannerCostsUIController.attachMessager(messager);
      footstepNodeCheckingUIController.attachMessager(messager);
      visibilityGraphsUIController.attachMessager(messager);
      dataExporterAnchorPaneController.attachMessager(messager);
      uiRobotController.attachMessager(messager);

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      bodyCollisionCheckingUIController.bindControls();
      footstepPlannerCostsUIController.bindControls();
      footstepNodeCheckingUIController.bindControls();
      visibilityGraphsUIController.bindControls();

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

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                                 StartPositionTopic, LowLevelGoalPositionTopic, GoalPositionTopic);
      this.startGoalOrientationViewer = new StartGoalOrientationViewer(messager);
      this.startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                         StartPositionTopic, GoalPositionTopic, PlanarRegionDataTopic, SelectedRegionTopic,
                                                         StartOrientationEditModeEnabledTopic, GoalOrientationEditModeEnabledTopic);
      this.nodeCheckerEditor = new NodeCheckerEditor(messager, subScene);
      this.orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene());
      this.pathViewer = new FootstepPathMeshViewer(messager);
      this.nodeCheckerRenderer = new NodeCheckerRenderer(messager, contactPointParameters);
      this.dataExporter = new FootstepPlannerDataExporter(messager);
      this.bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
      this.graphRenderer = new OccupancyMapRenderer(messager);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(nodeCheckerRenderer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
      view3dFactory.addNodeToView(graphRenderer.getRoot());

      if(fullHumanoidRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXRobotVisualizer(fullHumanoidRobotModelFactory);
         messager.registerTopicListener(RobotConfigurationDataTopic, robotVisualizer::submitNewConfiguration);
         mainTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
      }

      if(previewModelFactory == null)
      {
         walkingPreviewVisualizer = null;
      }
      else
      {
         recursivelyModifyGraphics(previewModelFactory.getRobotDescription().getChildrenJoints().get(0), YoAppearance.AliceBlue());
         walkingPreviewVisualizer = new JavaFXRobotVisualizer(previewModelFactory);
         walkingPreviewVisualizer.getRootNode().setMouseTransparent(true);
         view3dFactory.addNodeToView(walkingPreviewVisualizer.getRootNode());
         mainTabController.setPreviewModel(walkingPreviewVisualizer.getFullRobotModel());
         walkingPreviewVisualizer.getFullRobotModel().getRootJoint().setJointPosition(new Vector3D(Double.NaN, Double.NaN, Double.NaN));
         walkingPreviewVisualizer.start();
      }

      if(walkingControllerParameters != null)
      {
         mainTabController.setDefaultTiming(walkingControllerParameters.getDefaultSwingTime(), walkingControllerParameters.getDefaultTransferTime());
         pathViewer.setDefaultWaypointProportions(walkingControllerParameters.getSwingTrajectoryParameters().getSwingWaypointProportions());
      }

      if(contactPointParameters != null)
      {
         mainTabController.setContactPointParameters(contactPointParameters);
         pathViewer.setDefaultContactPoints(contactPointParameters);
      }

      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      nodeCheckerRenderer.start();
      nodeCheckerEditor.start();
      bodyPathMeshViewer.start();
      visibilityGraphsRenderer.start();
      graphRenderer.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public static void recursivelyModifyGraphics(JointDescription joint, AppearanceDefinition ghostApperance)
   {
      if (joint == null)
         return;
      LinkDescription link = joint.getLink();
      if (link == null)
         return;
      LinkGraphicsDescription linkGraphics = link.getLinkGraphics();

      if (linkGraphics != null)
      {
         ArrayList<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

         if (graphics3dInstructions == null)
            return;

         for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
         {
            if (primitive instanceof Graphics3DInstruction)
            {
               Graphics3DInstruction modelInstruction = (Graphics3DInstruction) primitive;
               modelInstruction.setAppearance(ghostApperance);
            }
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModifyGraphics(child, ghostApperance);
      }
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      uiRobotController.setRobotLowLevelMessenger(robotLowLevelMessenger);
   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      startGoalPositionViewer.stop();
      startGoalOrientationViewer.stop();
      startGoalEditor.stop();
      orientationEditor.stop();
      pathViewer.stop();
      nodeCheckerRenderer.stop();
      dataExporter.stop();
      bodyPathMeshViewer.stop();
      visibilityGraphsRenderer.stop();
      graphRenderer.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();
   }

   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager);
   }

   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager, SaveableFootstepPlannerParameters plannerParameters,
                                                    VisibilityGraphsParameters visibilityGraphsParameters,
                                                    FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                                                    FullHumanoidRobotModelFactory previewModelFactory,
                                                    RobotContactPointParameters<RobotSide> contactPointParameters,
                                                    WalkingControllerParameters walkingControllerParameters)
         throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullHumanoidRobotModelFactory, previewModelFactory,
                                   contactPointParameters, walkingControllerParameters);
   }
}
