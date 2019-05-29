package us.ihmc.quadrupedUI.video;

import controller_msgs.msg.dds.VideoPacket;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.effect.BlendMode;
import javafx.util.Duration;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;

public class QuadrupedVideoViewOverlay
{
   private final JavaFXROS2VideoView videoView;
   private Mode currentMode = Mode.MINIMIZED;

   private final DoubleProperty animationDurationProperty = new SimpleDoubleProperty(this, "animationDuration", 0.15);

   private enum Mode
   {
      MINIMIZED(200.0), MAXIMIZED(800.0);

      private final double width;

      private Mode(double width)
      {
         this.width = width;
      }

      public double getWidth()
      {
         return width;
      }
   }

   public QuadrupedVideoViewOverlay(int width, int height, boolean flipX, boolean flipY)
   {
      videoView = new JavaFXROS2VideoView(width, height, flipX, flipY);
      videoView.setPreserveRatio(true);
      videoView.setFitWidth(currentMode.getWidth());
      videoView.setStyle("-fx-effect: dropshadow(three-pass-box, rgba(0,0,0,0.6), 5, 0, 0, 0);");
      videoView.setBlendMode(BlendMode.OVERLAY);
   }

   public void toggleMode()
   {
      Mode endMode;

      switch (currentMode)
      {
      case MINIMIZED:
         endMode = Mode.MAXIMIZED;
         break;
      case MAXIMIZED:
         endMode = Mode.MINIMIZED;
         break;
      default:
         throw new RuntimeException("Unhandle mode: " + currentMode);
      }

      Timeline timeline = new Timeline();
      timeline.setAutoReverse(false);
      timeline.setCycleCount(1);
      timeline.getKeyFrames().add(new KeyFrame(Duration.ZERO, new KeyValue(videoView.fitWidthProperty(), videoView.getFitWidth())));
      timeline.getKeyFrames()
              .add(new KeyFrame(Duration.seconds(animationDurationProperty.get()), new KeyValue(videoView.fitWidthProperty(), endMode.getWidth())));
      timeline.setOnFinished(event -> currentMode = endMode);
      timeline.play();
   }

   public void start(Messager messager, Topic<VideoPacket> videoTopic)
   {
      videoView.start(messager, videoTopic);
   }

   public void stop()
   {
      videoView.stop();
   }

   public Node getNode()
   {
      return videoView;
   }
}
