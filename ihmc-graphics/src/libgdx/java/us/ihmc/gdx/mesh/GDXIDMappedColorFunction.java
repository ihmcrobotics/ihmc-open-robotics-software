package us.ihmc.gdx.mesh;

import com.badlogic.gdx.graphics.Color;

import java.util.ArrayList;
import java.util.function.Function;

public class GDXIDMappedColorFunction implements Function<Integer, Color>
{
   private final ArrayList<Color> colors = new ArrayList<>();

   public GDXIDMappedColorFunction()
   {
      colors.add(new Color(104.0f / 255.0f, 130.0f / 255.0f, 219.0f / 255.0f, 1.0f));
      colors.add(new Color(113.0f / 255.0f, 168.0f / 255.0f, 133.0f / 255.0f, 1.0f));
      colors.add(new Color(196.0f / 255.0f, 182.0f / 255.0f, 90.0f / 255.0f, 1.0f));
      colors.add(new Color(190.0f / 255.0f, 89.0f / 255.0f, 110.0f / 255.0f, 1.0f));
      colors.add(new Color(155.0f / 255.0f, 80.0f / 255.0f, 190.0f / 255.0f, 1.0f));
      colors.add(new Color(114.0f / 255.0f, 120.0f / 255.0f, 199.0f / 255.0f, 1.0f));
      colors.add(new Color(118.0f / 255.0f, 158.0f / 255.0f, 143.0f / 255.0f, 1.0f));
      colors.add(new Color(176.0f / 255.0f, 192.0f / 255.0f, 80.0f / 255.0f, 1.0f));
      colors.add(new Color(200.0f / 255.0f, 79.0f / 255.0f, 100.0f / 255.0f, 1.0f));
      colors.add(new Color(135.0f / 255.0f, 80.0f / 255.0f, 140.0f / 255.0f, 1.0f));
      colors.add(new Color(90.0f / 255.0f, 100.0f / 255.0f, 219.0f / 255.0f, 1.0f));
      colors.add(new Color(113.0f / 255.0f, 128.0f / 255.0f, 53.0f / 255.0f, 1.0f));
      colors.add(new Color(126.0f / 255.0f, 112.0f / 255.0f, 80.0f / 255.0f, 1.0f));
      colors.add(new Color(200.0f / 255.0f, 59.0f / 255.0f, 30.0f / 255.0f, 1.0f));
      colors.add(new Color(75.0f / 255.0f, 185.0f / 255.0f, 130.0f / 255.0f, 1.0f));
      colors.add(new Color(24.0f / 255.0f, 130.0f / 255.0f, 219.0f / 255.0f, 1.0f));
      colors.add(new Color(23.0f / 255.0f, 168.0f / 255.0f, 133.0f / 255.0f, 1.0f));
      colors.add(new Color(26.0f / 255.0f, 182.0f / 255.0f, 90.0f / 255.0f, 1.0f));
      colors.add(new Color(60.0f / 255.0f, 89.0f / 255.0f, 110.0f / 255.0f, 1.0f));
      colors.add(new Color(65.0f / 255.0f, 80.0f / 255.0f, 190.0f / 255.0f, 1.0f));
   }

   public Color getColor(int id)
   {
      return colors.get(Math.abs(id % colors.size()));
   }

   @Override
   public Color apply(Integer id)
   {
      return getColor(id);
   }
}
