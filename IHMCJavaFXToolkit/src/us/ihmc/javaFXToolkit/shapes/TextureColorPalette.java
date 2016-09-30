package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.image.Image;
import javafx.scene.paint.Color;

public interface TextureColorPalette
{

   float[] getTextureLocation(Color color);

   float[] getTextureLocation(double red, double green, double blue);

   float[] getTextureLocation(int red, int green, int blue);

   Image getColorPalette();

}