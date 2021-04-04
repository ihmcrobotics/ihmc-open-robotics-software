package us.ihmc.humanoidBehaviors.ui.video;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class JavaFXROS2VideoViewer
{
   private ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private final int width;
   private final int height;

   public JavaFXROS2VideoViewer(ROS2Topic<?> topic, int width, int height)
   {
      this(ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "video_viewer"), topic, width, height);
   }

   public JavaFXROS2VideoViewer(ROS2Node ros2Node, ROS2Topic<?> topic, int width, int height)
   {
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.width = width;
      this.height = height;
      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(this::buildAndStartUI);
   }

   private void buildAndStartUI()
   {
      Stage primaryStage = new Stage();

      JavaFXROS2VideoView ros2VideoView = new JavaFXROS2VideoView(ros2Node, topic, width, height, false, false);

      StackPane stackPaneNode = new StackPane(ros2VideoView);
      stackPaneNode.setPrefSize(width, height);
      Scene scene = new Scene(stackPaneNode);
      primaryStage.setOnCloseRequest((e) -> {
         ros2VideoView.stop();
         ros2Node.destroy();
      });
      primaryStage.setX(0);  // essentially monitor selection
      primaryStage.setY(0);
      primaryStage.initStyle(StageStyle.DECORATED);
      primaryStage.setScene(scene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.show();

      ros2VideoView.start();
   }

   public static void main(String[] args)
   {
      new JavaFXROS2VideoViewer(ROS2Tools.VIDEO, 1024, 544);
   }
}