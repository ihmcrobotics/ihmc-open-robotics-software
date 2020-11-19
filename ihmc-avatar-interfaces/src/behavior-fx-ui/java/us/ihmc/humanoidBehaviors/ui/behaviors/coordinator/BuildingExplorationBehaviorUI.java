package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI.*;

public class BuildingExplorationBehaviorUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;
   private final LivePlanarRegionsGraphic reaPlanarRegionsGraphic;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final FootstepPlanGraphic footstepPlanGraphic;
   private final BuildingExplorationGoalMouseListener goalMouseListener;
   private final ROS2Node ros2Node;

   @FXML
   private BuildingExplorationUIDashboardController buildingExplorationUIDashboardController;

   public BuildingExplorationBehaviorUI(Stage primaryStage,
                                        JavaFXMessager messager,
                                        DRCRobotModel robotModel,
                                        ROS2Node ros2Node,
                                        Messager behaviorMessager) throws Exception
   {
      this.primaryStage = primaryStage;
      this.ros2Node = ros2Node;
      primaryStage.setTitle(getClass().getSimpleName());

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    controller_msgs.msg.dds.RobotConfigurationData.class,
                                                    ControllerAPIDefinition.getOutputTopic(robotModel.getSimpleRobotName()),
                                                    s -> messager.submitMessage(BuildingExplorationBehaviorAPI.RobotConfigurationData, s.takeNextData()));

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addDefaultLighting();

      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      worldCoordinateSystem.setMouseTransparent(true);
      view3dFactory.addNodeToView(worldCoordinateSystem);

      robotVisualizer = new JavaFXRobotVisualizer(robotModel);
      reaPlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      new IHMCROS2Callback<>(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, reaPlanarRegionsGraphic::acceptPlanarRegions);
      footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      new IHMCROS2Callback<>(ros2Node, TraverseStairsBehaviorAPI.PLANNED_STEPS, footstepDataListMessage ->
            footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage)));
      goalMouseListener = new BuildingExplorationGoalMouseListener(ros2Node, messager, subScene);

      robotVisualizer.start();
      goalMouseListener.start();

      GoalGraphic goalGraphic = new GoalGraphic();
      view3dFactory.addNodeToView(goalGraphic);
      goalGraphic.setMouseTransparent(true);
      messager.registerTopicListener(Goal, newGoal ->
      {
         goalGraphic.setTranslateX(newGoal.getX());
         goalGraphic.setTranslateY(newGoal.getY());
         goalGraphic.setTranslateZ(newGoal.getZ());
      });

      messager.registerTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);
      view3dFactory.addNodeToView(reaPlanarRegionsGraphic);
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      view3dFactory.addNodeToView(footstepPlanGraphic);

      buildingExplorationUIDashboardController.bindControls(messager, ros2Node);
      mainPane.setCenter(subScene);

      Scene mainScene = new Scene(mainPane);
      primaryStage.setScene(mainScene);
      primaryStage.setWidth(1400);
      primaryStage.setHeight(950);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      reaPlanarRegionsGraphic.destroy();
      robotVisualizer.stop();
      ros2Node.destroy();
      footstepPlanGraphic.destroy();
   }

   private static class GoalGraphic extends Group
   {
      public GoalGraphic()
      {
         TextureColorPalette1D colorPalette = new TextureColorPalette1D();
         colorPalette.setHueBased(1.0, 1.0);
         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

         meshBuilder.addSphere(0.1f, Color.rgb(50, 50, 50));
         MeshView goalSphere = new MeshView(meshBuilder.generateMesh());
         goalSphere.setMaterial(meshBuilder.generateMaterial());
         goalSphere.setMaterial(new PhongMaterial( Color.rgb(50, 50, 50)));
         getChildren().add(goalSphere);

         meshBuilder.clear();
         meshBuilder.addCylinder(0.14f, 0.014, new Point3D(), new AxisAngle(1.0, 0.0, 0.0, 0.4), Color.RED);
         MeshView goalFuse = new MeshView(meshBuilder.generateMesh());
         goalFuse.setMaterial(meshBuilder.generateMaterial());
         goalFuse.setMaterial(new PhongMaterial(Color.RED));
         getChildren().add(goalFuse);
      }
   }
}
