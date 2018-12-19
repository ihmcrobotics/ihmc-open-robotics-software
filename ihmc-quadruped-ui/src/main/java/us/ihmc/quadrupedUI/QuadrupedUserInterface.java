package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

public class QuadrupedUserInterface
{
   private final RealtimeRos2Node ros2Node;

   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;
      String robotName = modelFactory.getRobotDescription().getName();

      ros2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, "ihmc_" + robotName + "_user_interface");

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      robotVisualizer = new JavaFXQuadrupedVisualizer(messager, modelFactory);

      createSubscribers(robotName);


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
      ros2Node.spin();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());

   }

   private void createSubscribers(String robotName)
   {
      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> handleNewConfigurationData(s.takeNextData()));
   }

   private void handleNewConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      robotVisualizer.submitNewConfiguration(robotConfigurationData);
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
   }
}
