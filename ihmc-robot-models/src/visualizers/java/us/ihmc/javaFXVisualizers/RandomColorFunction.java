package us.ihmc.javaFXVisualizers;

import javafx.scene.paint.Color;

import java.util.Random;
import java.util.function.Function;

public class RandomColorFunction implements Function<Integer, Color>
{
   private final Random random = new Random(235723095L);
   private final int dimmestValue;
   private final int brightnessHeight;

   public RandomColorFunction()
   {
      this(80, 140);
   }

   public RandomColorFunction(int dimmestValue, int brightestValue)
   {
      this.dimmestValue = dimmestValue;
      brightnessHeight = brightestValue - dimmestValue;
   }

   @Override
   public Color apply(Integer id)
   {
      return Color.rgb(dimmestValue + Math.abs(random.nextInt()) % brightnessHeight,
                       dimmestValue + Math.abs(random.nextInt()) % brightnessHeight,
                       dimmestValue + Math.abs(random.nextInt()) % brightnessHeight);
   }
}
