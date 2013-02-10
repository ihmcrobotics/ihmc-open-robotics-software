package us.ihmc.graphics3DAdapter.graphics.appearances;

import java.awt.Color;
import java.awt.Component;
import java.net.URL;

import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;


//http://www.wdvl.com/Graphics/Colour/  has some of the color names I'm using...
// http://cloford.com/resources/colours/500col.htm

public class YoAppearance
{

   public static AppearanceDefinition YoboticsTexture(Component comp)
   {
      URL fileURL = Graphics3DAdapter.class.getResource("images/yobotics.jpg");
      return Texture(fileURL, comp);
   }

   public static AppearanceDefinition Texture(URL fileURL, Component comp)
   {

      return new YoAppearanceTexture(fileURL, comp);

   }

   public static AppearanceDefinition EarthTexture(Component comp)
   {

      URL fileURL = Graphics3DAdapter.class.getResource("images/earth.jpg");
      return Texture(fileURL, comp);

   }

   public static AppearanceDefinition StoneTexture(Component comp)
   {

      URL fileURL = Graphics3DAdapter.class.getResource("images/stone.jpg");
      return Texture(fileURL, comp);
   }

   public static AppearanceDefinition PlaneMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      return mat;
   }

   public static AppearanceDefinition AluminumMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      return mat;
   }

   public static AppearanceDefinition BlackMetalMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(6.0f);
      mat.setAmbientColor(0.16f, 0.18f, 0.2f);
      return mat;
   }

   public static AppearanceDefinition FenceMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.4f, 0.4f, 0.4f);
      mat.setDiffuseColor(0.95f, 0.95f, 0.95f);
      mat.setShininess(2.0f);
      mat.setAmbientColor(0.45f, 0.45f, 0.45f);
      return mat;
   }

   public static AppearanceDefinition RGBColor(double red, double green, double blue)
   {
      return new YoAppearanceRGBColor(red, green, blue, 0.0);
   }

   public static AppearanceDefinition RGBColor(float red, float green, float blue)
   {
      return new YoAppearanceRGBColor(red, green, blue, 0.0);
   }

   public static AppearanceDefinition RGBColorFrom8BitInts(int red, int green, int blue)
   {
      return RGBColor(((red)) / 255.0, ((green)) / 255.0, ((blue)) / 255.0);
   }
   
   public static AppearanceDefinition RGBColorFromHex(int hex)
   {
      int red = (hex>>16) & 0xff;
      int green = (hex>>8) & 0xff;
      int blue = hex & 0xff;
      
      return RGBColorFrom8BitInts(red, green, blue);
   }

   public static AppearanceDefinition Color(Color color)
   {
      return Color(new Color3f(color));
   }
   
   public static AppearanceDefinition Color(Color3f color)
   {
      return RGBColor(color.getX(), color.getY(), color.getZ());
   }

   public static AppearanceDefinition Black()
   {
      return RGBColor(0.0f, 0.0f, 0.0f);
   }
   
   public static AppearanceDefinition DarkGray()
   {
      return RGBColorFrom8BitInts(66,66,66);
   }

   public static AppearanceDefinition White()
   {
      return RGBColor(1.0f, 1.0f, 1.0f);
   }

   public static AppearanceDefinition Red()
   {
      return RGBColor(1.0f, 0.0f, 0.0f);
   }

   /**
    * Some colors from: http://cloford.com/resources/colours/500col.htm
    */
   public static AppearanceDefinition Pink()
   {
      return RGBColorFrom8BitInts(255, 192, 203);
   }

   public static AppearanceDefinition Orange()
   {
      return RGBColorFrom8BitInts(255, 128, 0);
   }

   public static AppearanceDefinition Orchid()
   {
      return RGBColorFrom8BitInts(218, 112, 214);
   }

   public static AppearanceDefinition DarkRed()
   {
      return RGBColor(0.3f, 0.0f, 0.0f);
   }

   public static AppearanceDefinition Blue()
   {
      return RGBColor(0.0f, 0.0f, 1.0f);
   }

   public static AppearanceDefinition DarkBlue()
   {
      return RGBColor(0.0f, 0.0f, 0.3f);
   }

   public static AppearanceDefinition Brown()
   {
      return RGBColorFrom8BitInts(70, 40, 0);
   }

   public static AppearanceDefinition Green()
   {
      return RGBColor(0.0f, 1.0f, 0.0f);
   }

   public static AppearanceDefinition DarkGreen()
   {
      return RGBColorFrom8BitInts(0,78, 2);
   }

   public static AppearanceDefinition Silver()
   {
      return RGBColor(0.75f, 0.75f, 0.75f);
   }

   public static AppearanceDefinition Gray()
   {
      return RGBColorFrom8BitInts(117, 117, 117);
   }

   public static AppearanceDefinition Maroon()
   {
      return RGBColor(0.50f, 0.0f, 0.0f);
   }

   public static AppearanceDefinition Purple()
   {
      return RGBColor(0.50f, 0.0f, 0.50f);
   }
   

   public static AppearanceDefinition Fuchsia()
   {
      return RGBColor(1.0f, 0.0f, 1.0f);
   }

   public static AppearanceDefinition Olive()
   {
      return RGBColor(0.50f, 0.50f, 0.0f);
   }

   public static AppearanceDefinition Yellow()
   {
      return RGBColor(1.0f, 1.0f, 0.0f);
   }

   public static AppearanceDefinition Navy()
   {
      return RGBColor(0.0f, 0.0f, 0.50f);
   }

   public static AppearanceDefinition Teal()
   {
      return RGBColor(0.0f, 0.50f, 0.50f);
   }

   public static AppearanceDefinition Aqua()
   {
      return RGBColor(0.0f, 1.0f, 1.0f);
   }

   
   //TODO: Finish the list below from
   // From http://www.w3schools.com/html/html_colornames.asp
   public static AppearanceDefinition AliceBlue()
   {
      return RGBColorFromHex(0xF0F8FF);
   }
   
   public static AppearanceDefinition AntiqueWhite()
   {
      return RGBColorFromHex(0xFAEBD7);
   }
   
//   public static AppearanceDefinition Aqua()
//   {
//      return RGBColorFromHex(0x00FFFF);
//   }
   
   public static AppearanceDefinition Aquamarine()
   {
      return RGBColorFromHex(0x7FFFD4);
   }
   
   public static AppearanceDefinition Azure()
   {
      return RGBColorFromHex(0xF0FFFF);
   }
   
   public static AppearanceDefinition Beige()
   {
      return RGBColorFromHex(0xF5F5DC);
   }
   
   
   
   public static AppearanceDefinition Gold()
   {
      return RGBColorFromHex(0xFFD700);
   }
   
   public static AppearanceDefinition GoldenRod()
   {
      return RGBColorFromHex(0xDAA520);
   }
   
   
   
   
   
   
   public static AppearanceDefinition Transparent()
   {
      return new YoAppearanceTransparent();
   }

   public static void makeTransparent(AppearanceDefinition appearance, double f)
   {
      appearance.setTransparancy(f);
   }

   //   public static void makeTransparent(YoAppearanceDefinition app, float transparency)
   //   {
   //      TransparencyAttributes ta = new TransparencyAttributes();
   //      ta.setTransparency(transparency);
   //      ta.setTransparencyMode(TransparencyAttributes.BLENDED);
   //
   //      app.setTransparencyAttributes(ta);
   //   }
   //
   //   public static void makeLineDrawing(YoAppearanceDefinition app)
   //   {
   //      PolygonAttributes pa = new PolygonAttributes();
   //      pa.setPolygonMode(PolygonAttributes.POLYGON_LINE);
   //      app.setPolygonAttributes(pa);
   //   }

   /*
    * public static YoAppearanceDefinition Transparent() {
    * YoAppearanceDefinition ret = new YoAppearanceDefinition(); Material mat =
    * new Material(); mat.setAmbientColor(0.0f,0.0f,0.0f);
    * 
    * TransparencyAttributes ta = new TransparencyAttributes();
    * ta.setTransparency(0.5f);
    * ta.setTransparencyMode(TransparencyAttributes.BLENDED);
    * 
    * ret.setTransparencyAttributes(ta); ret.setMaterial(mat); return ret; }
    */

   // public YoYoAppearanceDefinition()
   // {
   // }
}
