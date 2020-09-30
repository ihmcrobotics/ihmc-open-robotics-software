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
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorCoordinator;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.slam.viewer.FootstepMeshViewer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorAPI.*;

public class BuildingExplorationBehaviorUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;
   private final PlanarRegionViewer planarRegionViewer;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final BuildingExplorationFootstepVisualizer footstepVisualizer;
   private final ROS2Node ros2Node;

   @FXML
   private BuildingExplorationUIDashboardController buildingExplorationUIDashboardController;

   public BuildingExplorationBehaviorUI(Stage primaryStage, JavaFXMessager messager, DRCRobotModel robotModel) throws Exception
   {
      this.primaryStage = primaryStage;
      primaryStage.setTitle(getClass().getSimpleName());

      BuildingExplorationBehaviorCoordinator behaviorCoordinator = new BuildingExplorationBehaviorCoordinator(robotModel.getSimpleRobotName(),
                                                                                                              DomainFactory.PubSubImplementation.FAST_RTPS);
      ros2Node = createMessageBindings(DomainFactory.PubSubImplementation.FAST_RTPS, robotModel.getSimpleRobotName(), messager, behaviorCoordinator);
      behaviorCoordinator.setStateChangedCallback(newState -> messager.submitMessage(CurrentState, newState));
      behaviorCoordinator.setDebrisDetectedCallback(() -> messager.submitMessage(DebrisDetected, true));
      behaviorCoordinator.setStairsDetectedCallback(() -> messager.submitMessage(StairsDetected, true));
      behaviorCoordinator.setDoorDetectedCallback(() -> messager.submitMessage(DoorDetected, true));
      messager.registerTopicListener(IgnoreDebris, ignore -> behaviorCoordinator.ignoreDebris());

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
      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegions, ShowRegions);
      footstepVisualizer = new BuildingExplorationFootstepVisualizer(ros2Node);

      robotVisualizer.start();
      planarRegionViewer.start();
      footstepVisualizer.start();

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
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      view3dFactory.addNodeToView(footstepVisualizer.getRoot());

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
      planarRegionViewer.stop();
      robotVisualizer.stop();
      ros2Node.destroy();
      footstepVisualizer.stop();
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

   private static ROS2Node createMessageBindings(DomainFactory.PubSubImplementation pubSubImplementation,
                                                 String robotName,
                                                 Messager messager,
                                                 BuildingExplorationBehaviorCoordinator behaviorCoordinator)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "BuildingExplorationBehaviorUI");
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           ROS2Tools.LIDAR_REA_REGIONS,
                                           s -> messager.submitMessage(BuildingExplorationBehaviorAPI.PlanarRegions,
                                                                       PlanarRegionMessageConverter.convertToPlanarRegionsList(s.takeNextData())));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    controller_msgs.msg.dds.RobotConfigurationData.class,
                                                    ControllerAPIDefinition.getOutputTopic(robotName),
                                                    s -> messager.submitMessage(BuildingExplorationBehaviorAPI.RobotConfigurationData, s.takeNextData()));

      AtomicReference<Point3D> goal = messager.createInput(Goal);

      messager.registerTopicListener(RequestedState, behaviorCoordinator::requestState);
      AtomicReference<BuildingExplorationStateName> requestedState = messager.createInput(RequestedState);

      messager.registerTopicListener(Start, s ->
      {
         LogTools.debug("Start requested in UI... starting behavior coordinator");
         behaviorCoordinator.setBombPosition(goal.get());
         behaviorCoordinator.requestState(requestedState.get());
         behaviorCoordinator.start();
      });
      messager.registerTopicListener(Stop, s -> behaviorCoordinator.stop());

      return ros2Node;
   }
}
