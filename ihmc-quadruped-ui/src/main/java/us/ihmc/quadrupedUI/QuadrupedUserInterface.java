package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXQuadrupedVisualizer;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.MainTabController;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedUI.uiControllers.RobotControlTabController;
import us.ihmc.quadrupedUI.uiControllers.ManualStepTabController;
import us.ihmc.quadrupedUI.uiControllers.XGaitSettingsController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI.RobotConfigurationDataTopic;

public class QuadrupedUserInterface
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;

   @FXML
   private MainTabController plannerTabController;
   @FXML
   private RobotControlTabController robotControlTabController;

   @FXML
   private XGaitSettingsController xGaitSettingsController;

   @FXML
   private ManualStepTabController manualStepTabController;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                 QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly xGaitSettings, YoVariableRegistry registry)
         throws Exception
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      plannerTabController.attachMessager(messager);
      robotControlTabController.attachMessager(messager);
      xGaitSettingsController.attachMessager(messager, xGaitSettings);
      manualStepTabController.attachMessager(messager, xGaitSettings);

      setPlannerTabTopics();

      plannerTabController.bindControls();
      robotControlTabController.bindControls();
      xGaitSettingsController.bindControls();
      manualStepTabController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      robotVisualizer = new JavaFXQuadrupedVisualizer(messager, modelFactory, QuadrupedUIMessagerAPI.RobotModelTopic);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, this::submitNewConfiguration);

      plannerTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());

      view3dFactory.addNodeToView(robotVisualizer.getRootNode());

      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);

      cameraTracking = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            FramePoint3D rootJointPosition = new FramePoint3D(robotVisualizer.getFullRobotModel().getRootJoint().getFrameAfterJoint());
            rootJointPosition.changeFrame(ReferenceFrame.getWorldFrame());
            rootJointOffset.setX(rootJointPosition.getX());
            rootJointOffset.setY(rootJointPosition.getY());
            rootJointOffset.setZ(rootJointPosition.getZ());
         }
      };


      robotVisualizer.start();
      cameraTracking.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void submitNewConfiguration(RobotConfigurationData configuration)
   {
      robotVisualizer.submitNewConfiguration(configuration);
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         robotVisualizer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private void setPlannerTabTopics()
   {
      plannerTabController.setPlannerTypeTopic(QuadrupedUIMessagerAPI.PlannerTypeTopic);
      plannerTabController.setPlannerRequestIdTopic(QuadrupedUIMessagerAPI.PlannerRequestIdTopic);
      plannerTabController.setReceivedPlanIdTopic(QuadrupedUIMessagerAPI.ReceivedPlanIdTopic);
      plannerTabController.setFootstepPlanTopic(QuadrupedUIMessagerAPI.ShowFootstepPlanTopic, QuadrupedUIMessagerAPI.FootstepPlanTopic);
      plannerTabController.setPlanarRegionDataTopic(QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      plannerTabController.setPlannerTimeTakenTopic(QuadrupedUIMessagerAPI.PlannerTimeTakenTopic);
      plannerTabController.setPlannerTimeoutTopic(QuadrupedUIMessagerAPI.PlannerTimeoutTopic);
      plannerTabController.setComputePathTopic(QuadrupedUIMessagerAPI.ComputePathTopic);
      plannerTabController.setAbortPlanningTopic(QuadrupedUIMessagerAPI.AbortPlanningTopic);
      plannerTabController.setAcceptNewPlanarRegionsTopic(QuadrupedUIMessagerAPI.AcceptNewPlanarRegionsTopic);
      plannerTabController.setPlanningResultTopic(QuadrupedUIMessagerAPI.PlanningResultTopic);
      plannerTabController.setPlannerStatusTopic(QuadrupedUIMessagerAPI.PlannerStatusTopic);
      plannerTabController.setPlannerHorizonLengthTopic(QuadrupedUIMessagerAPI.PlannerHorizonLengthTopic);
      plannerTabController.setStartGoalTopics(QuadrupedUIMessagerAPI.EditModeEnabledTopic, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                              QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic, QuadrupedUIMessagerAPI.InitialSupportQuadrantTopic,
                                              QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                              QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic);
      plannerTabController.setAssumeFlatGroundTopic(QuadrupedUIMessagerAPI.AssumeFlatGroundTopic);
      plannerTabController.setGlobalResetTopic(QuadrupedUIMessagerAPI.GlobalResetTopic);
      plannerTabController.setPlannerPlaybackFractionTopic(QuadrupedUIMessagerAPI.PlannerPlaybackFractionTopic);
      plannerTabController.setXGaitSettingsTopic(QuadrupedUIMessagerAPI.XGaitSettingsTopic);
      plannerTabController.setShowFootstepPreviewTopic(QuadrupedUIMessagerAPI.ShowFootstepPreviewTopic);
   }


   public static QuadrupedUserInterface createUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                                            QuadrupedPhysicalProperties physicalProperties, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                            YoVariableRegistry registry) throws Exception
   {
      return new QuadrupedUserInterface(primaryStage, messager, modelFactory, physicalProperties, xGaitSettings, registry);
   }
}
