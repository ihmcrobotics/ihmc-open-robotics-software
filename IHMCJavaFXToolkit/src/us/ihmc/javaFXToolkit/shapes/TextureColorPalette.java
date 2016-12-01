package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.image.Image;
import javafx.scene.paint.Color;

public interface TextureColorPalette
{
   public float[] getTextureLocation(Color color);

   public default float[] getTextureLocation(double red, double green, double blue)
   {
      return getTextureLocation((int) (255 * red), (int) (255 * green), (int) (255 * blue));
   }

   public default float[] getTextureLocation(int red, int green, int blue)
   {
      return getTextureLocation(Color.rgb(red, green, blue));
   }

   public Image getColorPalette();
}