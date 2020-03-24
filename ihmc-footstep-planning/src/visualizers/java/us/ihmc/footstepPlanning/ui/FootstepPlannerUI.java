package us.ihmc.footstepPlanning.ui;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

import java.util.ArrayList;

import controller_msgs.msg.dds.REAStateRequestMessage;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.footstepPlanning.ui.components.*;
import us.ihmc.footstepPlanning.ui.controllers.*;
import us.ihmc.footstepPlanning.ui.viewers.*;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
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
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final GoalOrientationViewer goalOrientationViewer;
   private final FootstepPathMeshViewer pathViewer;
   private final FootstepPostProcessingMeshViewer postProcessingViewer;
   private final GoalOrientationEditor orientationEditor;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
   private final OccupancyMapRenderer occupancyMapRenderer;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final JavaFXRobotVisualizer walkingPreviewVisualizer;
   private final FootstepPlannerLogRenderer footstepPlannerLogRenderer;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private FootstepPostProcessingParametersUIController footstepPostProcessingParametersUIController;
   @FXML
   private FootstepPlannerLogVisualizerController footstepPlannerLogVisualizerController;
   @FXML
   private MainTabController mainTabController;
   @FXML
   private UIRobotController uiRobotController;

   @FXML
   private VisualizationController visibilityGraphsUIController;

   public FootstepPlannerUI(Stage primaryStage, FootstepPlannerParametersBasics plannerParameters, VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            FootstepPostProcessingParametersBasics footstepPostProcessingParameters) throws Exception
   {
      this(primaryStage, new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API), plannerParameters, visibilityGraphsParameters,
           footstepPostProcessingParameters, null, null, null);
      messager.startMessager();
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this(primaryStage, messager, new DefaultFootstepPlannerParameters(), new DefaultVisibilityGraphParameters(), new DefaultFootstepPostProcessingParameters(),
           null, null, null);
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParametersBasics plannerParameters,
                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            FootstepPostProcessingParametersBasics footstepPostProcessingParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            RobotContactPointParameters<RobotSide> contactPointParameters, WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this(primaryStage, messager, plannerParameters, visibilityGraphsParameters, footstepPostProcessingParameters, fullHumanoidRobotModelFactory, null,
           contactPointParameters, walkingControllerParameters);
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParametersBasics plannerParameters,
                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            FootstepPostProcessingParametersBasics footstepPostProcessingParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            FullHumanoidRobotModelFactory previewModelFactory, RobotContactPointParameters<RobotSide> contactPointParameters,
                            WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      footstepPlannerParametersUIController.setPlannerParameters(plannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);
      footstepPostProcessingParametersUIController.setPostProcessingParameters(footstepPostProcessingParameters);

      mainTabController.attachMessager(messager);
      footstepPlannerMenuUIController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      footstepPostProcessingParametersUIController.attachMessager(messager);
      footstepPlannerLogVisualizerController.attachMessager(messager);
      visibilityGraphsUIController.attachMessager(messager);
      uiRobotController.attachMessager(messager);

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      footstepPostProcessingParametersUIController.bindControls();
      footstepPlannerLogVisualizerController.bindControls();
      visibilityGraphsUIController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionData, ShowPlanarRegions);
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, null, GoalPositionEditModeEnabled,
                                                                 null, LowLevelGoalPosition, GoalMidFootPosition);
      this.goalOrientationViewer = new GoalOrientationViewer(messager);
      this.goalOrientationViewer.setPlannerParameters(plannerParameters);
      this.startGoalEditor = new StartGoalPositionEditor(messager, subScene, null, GoalPositionEditModeEnabled,
                                                         null, GoalMidFootPosition, PlanarRegionData, SelectedRegion,
                                                         null, GoalOrientationEditModeEnabled);
      this.orientationEditor = new GoalOrientationEditor(messager, view3dFactory.getSubScene());
      this.pathViewer = new FootstepPathMeshViewer(messager);
      this.postProcessingViewer = new FootstepPostProcessingMeshViewer(messager);
      this.bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
      this.occupancyMapRenderer = new OccupancyMapRenderer(messager);
      this.footstepPlannerLogRenderer = new FootstepPlannerLogRenderer(contactPointParameters, messager);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(goalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(postProcessingViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
      view3dFactory.addNodeToView(occupancyMapRenderer.getRoot());
      view3dFactory.addNodeToView(footstepPlannerLogRenderer.getRoot());

      if(fullHumanoidRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXRobotVisualizer(fullHumanoidRobotModelFactory);
         messager.registerTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);
         mainTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
         messager.registerTopicListener(ShowRobot, show -> robotVisualizer.getRootNode().setVisible(show));
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
         goalOrientationViewer.setDefaultContactPoints(contactPointParameters);
         footstepPlannerLogVisualizerController.setContactPointParameters(contactPointParameters);
      }

      planarRegionViewer.start();
      startGoalPositionViewer.start();
      goalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      postProcessingViewer.start();
      bodyPathMeshViewer.start();
      visibilityGraphsRenderer.start();
      occupancyMapRenderer.start();
      footstepPlannerLogRenderer.start();
      new FootPoseFromMidFootUpdater(messager).start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      mainScene.getStylesheets().add("us/ihmc/footstepPlanning/ui/FootstepPlannerUI.css");

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

   public void setREAStateRequestPublisher(IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher)
   {
      uiRobotController.setREAStateRequestPublisher(reaStateRequestPublisher);
   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      primaryStage.show();
      footstepPlannerLogVisualizerController.setup();
      footstepPlannerParametersUIController.setup();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      startGoalPositionViewer.stop();
      goalOrientationViewer.stop();
      startGoalEditor.stop();
      orientationEditor.stop();
      pathViewer.stop();
      postProcessingViewer.stop();
      bodyPathMeshViewer.stop();
      visibilityGraphsRenderer.stop();
      occupancyMapRenderer.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();
   }


   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager);
   }

   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParametersBasics plannerParameters,
                                                    VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                                    FootstepPostProcessingParametersBasics postProcessingParameters,
                                                    FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                                                    FullHumanoidRobotModelFactory previewModelFactory,
                                                    RobotContactPointParameters<RobotSide> contactPointParameters,
                                                    WalkingControllerParameters walkingControllerParameters)
         throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, postProcessingParameters,
                                   fullHumanoidRobotModelFactory, previewModelFactory, contactPointParameters, walkingControllerParameters);
   }
}
