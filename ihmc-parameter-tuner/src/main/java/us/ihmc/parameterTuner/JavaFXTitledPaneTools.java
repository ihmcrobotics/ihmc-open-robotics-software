package us.ihmc.parameterTuner;

import java.lang.reflect.Field;

import javafx.util.Duration;

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
         // FIXME The field is not accessible anymore. Best online suggestion is to create a skin.
//         durationField.set(TitledPaneSkin.TRANSITION_DURATION, millis);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }
}
