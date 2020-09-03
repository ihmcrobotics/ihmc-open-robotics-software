package us.ihmc.humanoidBehaviors.ui.video;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.Node;
import javafx.scene.effect.BlendMode;
import javafx.scene.input.MouseEvent;
import javafx.util.Duration;

public class JavaFXROS2VideoViewOverlay
{
   private final JavaFXVideoView videoView;
   private SizeMode currentMode = SizeMode.MIN;

   private final DoubleProperty animationDurationProperty = new SimpleDoubleProperty(this, "animationDuration", 0.15);

   private enum SizeMode
   {
      MIN(200.0), MED(800.0), MAX(1600.0);

      private final double width;
      private SizeMode(double width)
      {
         this.width = width;
      }

      public double getWidth()
      {
         return width;
      }
   }

   public JavaFXROS2VideoViewOverlay(JavaFXVideoView videoView)
   {
      this.videoView = videoView;

      videoView.setPreserveRatio(true);
      videoView.setFitWidth(currentMode.getWidth());
      videoView.setStyle("-fx-effect: dropshadow(three-pass-box, rgba(0,0,0,0.6), 5, 0, 0, 0);");
      videoView.setOpacity(0.5);
      videoView.setBlendMode(BlendMode.OVERLAY);

      videoView.addEventHandler(MouseEvent.MOUSE_ENTERED, e -> transitionBlending(1.0, null));
      videoView.addEventHandler(MouseEvent.MOUSE_EXITED, e -> transitionBlending(0.5, BlendMode.OVERLAY));
   }

   private void transitionBlending(double endOpacity, BlendMode endBlendMode)
   {
      Timeline timeline = new Timeline();
      timeline.setAutoReverse(false);
      timeline.setCycleCount(1);
      timeline.getKeyFrames().add(new KeyFrame(Duration.ZERO, new KeyValue(videoView.opacityProperty(), videoView.getOpacity())));
      timeline.getKeyFrames().add(new KeyFrame(Duration.seconds(animationDurationProperty.get()), new KeyValue(videoView.opacityProperty(), endOpacity)));
      timeline.setOnFinished(e2 -> videoView.setBlendMode(endBlendMode));
      timeline.play();
   }

   public void toggleMode()
   {
      SizeMode endMode;

      switch (currentMode)
      {
      case MIN:
         endMode = SizeMode.MED;
         break;
      case MED:
         endMode = SizeMode.MIN;
         break;
//      case MAX:
//         endMode = SizeMode.MIN;
//         break;
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

   public void start()
   {
      videoView.start();
   }

   public void stop()
   {
      videoView.stop();
   }

   public void destroy()
   {
      videoView.destroy();
   }

   public Node getNode()
   {
      return videoView;
   }
}
