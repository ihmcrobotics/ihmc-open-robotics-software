package us.ihmc.jme;

import com.jme3.math.ColorRGBA;
import javafx.scene.paint.Color;

public class JMEColorConversions
{
   public static ColorRGBA toJMEColor(javafx.scene.paint.Color color)
   {
      return new ColorRGBA((float) color.getRed(), (float) color.getGreen(), (float) color.getBlue(), (float) color.getOpacity());
   }

   public static javafx.scene.paint.Color toJavaFXColor(ColorRGBA color)
   {
      return new Color(color.getRed(), color.getGreen(), color.getBlue(), color.getAlpha());
   }
}