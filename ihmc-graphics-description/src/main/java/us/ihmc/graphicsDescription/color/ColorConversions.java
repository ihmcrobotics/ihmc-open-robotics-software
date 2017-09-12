package us.ihmc.graphicsDescription.color;

public class ColorConversions
{
   public static HSVValue awtToHSVValue(java.awt.Color color)
   {
      float[] hsvArray = new float[3];
      java.awt.Color.RGBtoHSB(color.getRed(), color.getGreen(), color.getBlue(), hsvArray);
      float hue = hsvArray[0] * 180.0f;
      float saturation = hsvArray[1] * 255.0f;
      float brightnessValue = hsvArray[2] * 255.0f;
      return new HSVValue(hue, saturation, brightnessValue);
   }

   public static java.awt.Color hsvValueToAwt(HSVValue hsvValue)
   {
      int rgb = java.awt.Color.HSBtoRGB((float) hsvValue.getHue() / 180.0f, (float) hsvValue.getSaturation() / 255.0f, (float) hsvValue.getBrightnessValue() / 255.0f);
      int red = (rgb >> 16) & 0xFF;
      int green = (rgb >> 8) & 0xFF;
      int blue = (rgb) & 0xFF;
      return new java.awt.Color(red, green, blue);
   }

   public static java.awt.Color jfxToAwt(javafx.scene.paint.Color jfxColor)
   {
      return new java.awt.Color((float) jfxColor.getRed(), (float) jfxColor.getGreen(), (float) jfxColor.getBlue(), (float) jfxColor.getOpacity());
   }
   
   public static javafx.scene.paint.Color awtToJfx(java.awt.Color awtColor)
   {
      return javafx.scene.paint.Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue(), awtColor.getAlpha() / 255.0);
   }
}
