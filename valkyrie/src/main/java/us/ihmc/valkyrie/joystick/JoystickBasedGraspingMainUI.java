package us.ihmc.valkyrie.joystick;

import java.io.IOException;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.ui.JavaFXPlanarRegionsViewer;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;

public class JoystickBasedGraspingMainUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final GraspingJavaFXController graspingJavaFXController;
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;
   private final JavaFXPlanarRegionsViewer planarRegionsViewer;

   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(GraspingJavaFXTopics.API);
   private final XBoxOneJavaFXController xBoxOneJavaFXController;

   @FXML
   private GraspingPaneController graspingPaneController;

   public JoystickBasedGraspingMainUI(String robotName, Stage primaryStage, Ros2Node ros2Node, FullHumanoidRobotModelFactory fullRobotModelFactory,
                                      HandFingerTrajectoryMessagePublisher handFingerTrajectoryMessagePublisher)
         throws Exception
   {
      this.primaryStage = primaryStage;
      xBoxOneJavaFXController = new XBoxOneJavaFXController(messager);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      LogTools.info(""+getClass().getResource(getClass().getSimpleName()));
      LogTools.info(""+getClass().getSimpleName());
      LogTools.info(""+getClass());
      LogTools.info(""+getClass().getName());
      LogTools.info(""+getClass().getResource(getClass().getName()));
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();

      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      javaFXRobotVisualizer = new JavaFXRobotVisualizer(fullRobotModelFactory);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                           s -> javaFXRobotVisualizer.submitNewConfiguration(s.takeNextData()));
      view3dFactory.addNodeToView(javaFXRobotVisualizer.getRootNode());

      planarRegionsViewer = new JavaFXPlanarRegionsViewer();
      ROS2Tools.createCallbackSubscription(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> planarRegionsViewer.submitPlanarRegions(s.takeNextData()));
      view3dFactory.addNodeToView(planarRegionsViewer.getRootNode());
      
      graspingJavaFXController = new GraspingJavaFXController(robotName, messager, ros2Node, fullRobotModelFactory, javaFXRobotVisualizer,
                                                              handFingerTrajectoryMessagePublisher);
      view3dFactory.addNodeToView(graspingJavaFXController.getRootNode());

      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);

      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);
      Translate manipulationFocusPoint = new Translate(0.0, 0.0, 1.0);
      cameraController.prependTransform(manipulationFocusPoint);

      messager.startMessager();

      graspingPaneController.initialize(messager);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setScene(new Scene(mainPane, 800, 600));
      primaryStage.setOnCloseRequest(event -> stop());

      start();
   }

   public void start() throws IOException
   {
      primaryStage.show();
      javaFXRobotVisualizer.start();
      planarRegionsViewer.start();
      graspingJavaFXController.start();
   }

   public void stop()
   {
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      xBoxOneJavaFXController.stop();
      javaFXRobotVisualizer.stop();
      planarRegionsViewer.stop();
      graspingJavaFXController.stop();

      ThreadTools.sleep(100); // Give some time to send the message.:
   }
}