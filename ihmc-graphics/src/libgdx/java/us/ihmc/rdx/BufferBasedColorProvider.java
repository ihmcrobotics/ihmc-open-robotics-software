package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.rdx.RDXPointCloudRenderer.ColorProvider;

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

      return colors.get(indexR++).r;
   }

   @Override
   public float getNextG()
   {
      if (colors.size() <= indexG)
         return 0;

      return colors.get(indexG++).g;
   }

   @Override
   public float getNextB()
   {
      if (colors.size() <= indexB)
         return 0;

      return colors.get(indexB++).b;
   }

   @Override
   public float getNextA()
   {
      if (colors.size() <= indexG)
         return 0;

      return colors.get(indexG++).a;
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
