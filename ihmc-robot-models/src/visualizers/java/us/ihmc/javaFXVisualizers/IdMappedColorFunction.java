package us.ihmc.javaFXVisualizers;

import javafx.scene.paint.Color;

import java.util.ArrayList;
import java.util.function.Function;

public class IdMappedColorFunction implements Function<Integer, Color>
{
   public static final IdMappedColorFunction INSTANCE = new IdMappedColorFunction();

   private static final ArrayList<Color> colors = new ArrayList<>();

   static
   {
      colors.add(Color.rgb(104, 130, 219));
      colors.add(Color.rgb(113, 168, 133));
      colors.add(Color.rgb(196, 182, 90));
      colors.add(Color.rgb(190, 89, 110));
      colors.add(Color.rgb(155, 80, 190));
      colors.add(Color.rgb(114, 120, 199));
      colors.add(Color.rgb(118, 158, 143));
      colors.add(Color.rgb(176, 192, 80));
      colors.add(Color.rgb(200, 79, 100));
      colors.add(Color.rgb(135, 80, 140));
      colors.add(Color.rgb(90, 100, 219));
      colors.add(Color.rgb(113, 128, 53));
      colors.add(Color.rgb(126, 112, 80));
      colors.add(Color.rgb(200, 59, 30));
      colors.add(Color.rgb(75, 185, 130));
      colors.add(Color.rgb(24, 130, 219));
      colors.add(Color.rgb(23, 168, 133));
      colors.add(Color.rgb(26, 182, 90));
      colors.add(Color.rgb(60, 89, 110));
      colors.add(Color.rgb(65, 80, 190));
   }

   @Override
   public Color apply(Integer id)
   {
      return colors.get(Math.abs(id % colors.size()));
   }
}
