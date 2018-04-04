package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.Optional;

import javafx.geometry.Insets;
import javafx.scene.control.Button;
import javafx.scene.control.TextInputDialog;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import us.ihmc.parameterTuner.guiElements.ParameterChangeListener;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;
import us.ihmc.robotics.sliderboard.Sliderboard;

public class SliderButton extends Button
{
   private final ImageView sliderGraphic;
   private boolean isActive = false;

   private ParameterChangeListener listener;
   private int sliderIndex = -1;

   private Tuner tuner;
   private Sliderboard sliderboard;

   public SliderButton()
   {
      Image sliderImage = new Image(SliderButton.class.getResourceAsStream("/sliders.png"));
      sliderGraphic = new ImageView(sliderImage);

      setPadding(new Insets(TuningBox.buttonPadding));
      sliderGraphic.setFitWidth(TuningBox.imageSize);
      sliderGraphic.setFitHeight(TuningBox.imageSize);
      setGraphic(sliderGraphic);

      double size = TuningBox.imageSize + 2.0 * TuningBox.buttonPadding;
      setPrefSize(size, size);
      setMinSize(size, size);
      setMaxSize(size, size);

      setOnAction(event -> {
         if (!isActive)
         {
            connect();
         }
         else
         {
            disconnect(true);
         }
      });
   }

   private void disconnect(boolean discardIndex)
   {
      setText(null);
      setGraphic(sliderGraphic);
      isActive = false;

      if (sliderboard != null && sliderIndex != -1)
      {
         sliderboard.clearListeners(sliderIndex);
      }
      if (tuner != null && listener != null)
      {
         tuner.removeChangeListener(listener);
      }

      if (discardIndex)
      {
         sliderIndex = -1;
      }
   }

   private void connect()
   {
      TextInputDialog dialog = new TextInputDialog("1");
      dialog.setTitle("Slider Index");
      dialog.setHeaderText("Select Slider Index");
      dialog.setContentText("Enter the index:");

      Optional<String> result = dialog.showAndWait();
      if (!result.isPresent())
      {
         return;
      }

      try
      {
         int sliderIndex = Integer.parseInt(result.get());
         connect(sliderIndex);
      }
      catch (NumberFormatException e)
      {
         return;
      }
   }

   private void connect(int sliderIndex)
   {
      if (isActive)
      {
         disconnect(true);
      }

      if (sliderIndex > 99)
      {
         return;
      }

      if (sliderboard.addListener(tuner, sliderIndex))
      {
         setGraphic(null);
         setText(Integer.toString(sliderIndex));

         listener = p -> updateSlider(sliderIndex);
         tuner.addChangeListener(listener);
         updateSlider(sliderIndex);

         this.sliderIndex = sliderIndex;
         isActive = true;
      }
   }

   private void updateSlider(int sliderIndex)
   {
      double sliderPercent = tuner.getValuePercent();
      sliderboard.setSliderValue(sliderPercent, sliderIndex);
   }

   public void link(Tuner tuner, Sliderboard sliderboard)
   {
      this.tuner = tuner;
      this.sliderboard = sliderboard;

      // Reconnect on Tab switch.
      if (sliderIndex != -1)
      {
         connect(sliderIndex);
      }
   }

   public void unlink()
   {
      disconnect(false);

      this.tuner = null;
      this.sliderboard = null;
   }
}
