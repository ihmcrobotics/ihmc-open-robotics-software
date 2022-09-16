package us.ihmc.quadrupedUI.video;

import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

public class QuadrupedJavaFXROS2VideoViewer extends ApplicationNoModule
{
   private static final int width = 1024;
   private static final int height = 544;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "video_viewer");

      QuadrupedJavaFXROS2VideoView ros2VideoView = new QuadrupedJavaFXROS2VideoView(width, height, false, false);

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

      ros2VideoView.start(ros2Node);
   }

   @Override
   public void stop() throws Exception
   {
      LogTools.info("JavaFX stop() called");
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}