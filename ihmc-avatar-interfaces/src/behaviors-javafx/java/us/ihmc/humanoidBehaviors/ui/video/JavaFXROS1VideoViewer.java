package us.ihmc.humanoidBehaviors.ui.video;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import java.net.URI;

public class JavaFXROS1VideoViewer
{
   // Realsense 640, 480
   // Multisense 1024, 544
   private static final int width = 1024;
   private static final int height = 544;
   private RosMainNode ros1Node;

   public JavaFXROS1VideoViewer(RosMainNode ros1Node)
   {
      this.ros1Node = ros1Node;
      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(this::buildAndStartUI);
   }

   private void buildAndStartUI()
   {
      Stage primaryStage = new Stage();

      JavaFXROS1VideoView ros1VideoView = new JavaFXROS1VideoView(ros1Node, RosTools.MULTISENSE_VIDEO, width, height, false, false);

      StackPane stackPaneNode = new StackPane(ros1VideoView);
      stackPaneNode.setPrefSize(width, height);
      Scene scene = new Scene(stackPaneNode);
      primaryStage.setOnCloseRequest((e) -> {
         ros1VideoView.stop();
      });
      primaryStage.setX(0);  // essentially monitor selection
      primaryStage.setY(0);
      primaryStage.initStyle(StageStyle.DECORATED);
      primaryStage.setScene(scene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.show();

      ros1VideoView.start();
   }

   public static void main(String[] args)
   {
      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "video_viewer", true);

      new JavaFXROS1VideoViewer(ros1Node);
      ros1Node.execute();
   }
}