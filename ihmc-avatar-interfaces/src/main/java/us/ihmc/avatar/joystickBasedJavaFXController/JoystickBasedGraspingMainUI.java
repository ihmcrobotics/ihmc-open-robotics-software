package us.ihmc.avatar.joystickBasedJavaFXController;

import java.io.IOException;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.Ros2Node;

public class JoystickBasedGraspingMainUI
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final GraspingJavaFXController graspingJavaFXController;
   private final JavaFXRobotVisualizer javaFXRobotVisualizer;

   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(GraspingJavaFXTopics.API);
   private final XBoxOneJavaFXController xBoxOneJavaFXController;

   private final AnimationTimer cameraTracking;

   @FXML
   private GraspingPaneController graspingPaneController;

   public JoystickBasedGraspingMainUI(String robotName, Stage primaryStage, Ros2Node ros2Node, FullHumanoidRobotModelFactory fullRobotModelFactory)
         throws Exception
   {
      this.primaryStage = primaryStage;
      xBoxOneJavaFXController = new XBoxOneJavaFXController(messager);

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
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

      graspingJavaFXController = new GraspingJavaFXController(robotName, messager, ros2Node, fullRobotModelFactory, javaFXRobotVisualizer);
      view3dFactory.addNodeToView(graspingJavaFXController.getRootNode());

      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);

      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);

      cameraTracking = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
         }
      };

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
      graspingJavaFXController.start();
      cameraTracking.start();
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
      graspingJavaFXController.stop();
      cameraTracking.stop();

      ThreadTools.sleep(100); // Give some time to send the message.:
   }
}