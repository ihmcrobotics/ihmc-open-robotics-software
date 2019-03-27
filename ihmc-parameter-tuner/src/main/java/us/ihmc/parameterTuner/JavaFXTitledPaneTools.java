package us.ihmc.parameterTuner;

import java.lang.reflect.Field;

import com.sun.javafx.scene.control.skin.TitledPaneSkin;

import javafx.util.Duration;

@SuppressWarnings("restriction")
public class JavaFXTitledPaneTools
{
   /**
    * Hacky way of speeding up the tilted plane transition via reflection.
    */
   public static void setTitledPaneAnimationTime(int millis)
   {
      try
      {
         Field durationField = Duration.class.getDeclaredField("millis");
         durationField.setAccessible(true);
         durationField.set(TitledPaneSkin.TRANSITION_DURATION, millis);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }
}
