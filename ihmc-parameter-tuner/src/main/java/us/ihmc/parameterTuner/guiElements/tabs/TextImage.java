package us.ihmc.parameterTuner.guiElements.tabs;

import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.layout.StackPane;
import javafx.scene.text.Text;

public class TextImage
{

   public static Image create(String text)
   {
      Text t = new Text(text);
      new Scene(new StackPane(t));
      return t.snapshot(null, null);
   }

}
