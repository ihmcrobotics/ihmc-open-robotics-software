package us.ihmc.jme;

import com.jme3.math.ColorRGBA;

public class JMEColorConversions
{
   public static ColorRGBA toJMEColor(javafx.scene.paint.Color color)
   {
      return new ColorRGBA((float) color.getRed(), (float) color.getGreen(), (float) color.getBlue(), (float) color.getOpacity());
   }
}