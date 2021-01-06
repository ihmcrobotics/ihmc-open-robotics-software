package us.ihmc.gdx.mesh;

import com.badlogic.gdx.graphics.Color;

import java.util.ArrayList;
import java.util.function.Function;

public class GDXIDMappedColorFunction implements Function<Integer, Color>
{
   public static final GDXIDMappedColorFunction INSTANCE = new GDXIDMappedColorFunction();

   private static final ArrayList<Color> colors = new ArrayList<>();

   static
   {
      colors.add(Color.rgba8888(104.0f, 130.0f, 219.0f, 1.0f));
      colors.add(Color.rgba8888(113, 168, 133));
      colors.add(Color.rgba8888(196, 182, 90));
      colors.add(Color.rgba8888(190, 89, 110));
      colors.add(Color.rgba8888(155, 80, 190));
      colors.add(Color.rgba8888(114, 120, 199));
      colors.add(Color.rgba8888(118, 158, 143));
      colors.add(Color.rgba8888(176, 192, 80));
      colors.add(Color.rgba8888(200, 79, 100));
      colors.add(Color.rgba8888(135, 80, 140));
      colors.add(Color.rgba8888(90, 100, 219));
      colors.add(Color.rgba8888(113, 128, 53));
      colors.add(Color.rgba8888(126, 112, 80));
      colors.add(Color.rgba8888(200, 59, 30));
      colors.add(Color.rgba8888(75, 185, 130));
      colors.add(Color.rgba8888(24, 130, 219));
      colors.add(Color.rgba8888(23, 168, 133));
      colors.add(Color.rgba8888(26, 182, 90));
      colors.add(Color.rgba8888(60, 89, 110));
      colors.add(Color.rgba8888(65, 80, 190));
   }

   @Override
   public Color apply(Integer id)
   {
      return colors.get(Math.abs(id % colors.size()));
   }
}
