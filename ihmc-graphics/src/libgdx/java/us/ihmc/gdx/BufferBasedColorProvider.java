package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.gdx.GDXPointCloudRenderer.ColorProvider;

import java.util.ArrayList;

public class BufferBasedColorProvider implements ColorProvider
{
   private ArrayList<Color> colors = new ArrayList<>();
   private int indexR = 0;
   private int indexG = 0;
   private int indexB = 0;

   @Override
   public float getNextR()
   {
      if (colors.size() <= indexR)
         return 0;

      Color color = colors.get(indexR++);
      return colors == null ? 0 : color.r;
   }

   @Override
   public float getNextG()
   {
      if (colors.size() <= indexG)
         return 0;

      Color color = colors.get(indexG++);
      return colors == null ? 0 : color.g;
   }

   @Override
   public float getNextB()
   {
      if (colors.size() <= indexB)
         return 0;

      Color color = colors.get(indexB++);
      return colors == null ? 0 : color.b;
   }

   public void resetIndex() {
      indexR = 0;
      indexG = 0;
      indexB = 0;
   }

   public void add(Color color) {
      colors.add(color);
   }
}
