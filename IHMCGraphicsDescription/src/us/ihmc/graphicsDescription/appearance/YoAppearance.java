package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.util.Random;

import javax.vecmath.Color3f;


//http://www.wdvl.com/Graphics/Colour/  has some of the color names I'm using...
// http://cloford.com/resources/colours/500col.htm

public class YoAppearance
{
   public static AppearanceDefinition YoboticsTexture()
   {
      return Texture("images/yobotics.jpg");
   }

   public static AppearanceDefinition Texture(String path)
   {
      return new YoAppearanceTexture(path);

   }

   public static AppearanceDefinition Texture(BufferedImage bufferedImage)
   {
      return new YoAppearanceTexture(bufferedImage);
   }

   public static AppearanceDefinition EarthTexture()
   {
      return Texture("images/earth.jpg");
   }

   public static AppearanceDefinition StoneTexture()
   {
      return Texture("images/stone.jpg");
   }

   public static AppearanceDefinition[] getStandardRoyGBivRainbow()
   {
      AppearanceDefinition[] rainbow = new AppearanceDefinition[] { YoAppearance.Red(), YoAppearance.OrangeRed(), YoAppearance.Yellow(),
            YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Indigo(), YoAppearance.Purple()};
      
      return rainbow;
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
   
   public static AppearanceDefinition RGBColor(double red, double green, double blue, double transparency)
   {
      return new YoAppearanceRGBColor(red, green, blue, transparency);
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
      int red = (hex >> 16) & 0xff;
      int green = (hex >> 8) & 0xff;
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

   // From http://www.w3schools.com/html/html_colornames.asp
   public static AppearanceDefinition AliceBlue()
   {
      return RGBColorFromHex(0xF0F8FF);
   }

   public static AppearanceDefinition AntiqueWhite()
   {
      return RGBColorFromHex(0xFAEBD7);
   }

   public static AppearanceDefinition Aqua()
   {
      return RGBColorFromHex(0x00FFFF);
   }

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

   public static AppearanceDefinition Bisque()
   {
      return RGBColorFromHex(0xFFE4C4);
   }

   public static AppearanceDefinition Black()
   {
      return RGBColorFromHex(0x000000);
   }

   public static AppearanceDefinition BlanchedAlmond()
   {
      return RGBColorFromHex(0xFFEBCD);
   }

   public static AppearanceDefinition Blue()
   {
      return RGBColorFromHex(0x0000FF);
   }

   public static AppearanceDefinition BlueViolet()
   {
      return RGBColorFromHex(0x8A2BE2);
   }

   public static AppearanceDefinition Brown()
   {
      return RGBColorFromHex(0xA52A2A);
   }

   public static AppearanceDefinition BurlyWood()
   {
      return RGBColorFromHex(0xDEB887);
   }

   public static AppearanceDefinition CadetBlue()
   {
      return RGBColorFromHex(0x5F9EA0);
   }

   public static AppearanceDefinition Chartreuse()
   {
      return RGBColorFromHex(0x7FFF00);
   }

   public static AppearanceDefinition Chocolate()
   {
      return RGBColorFromHex(0xD2691E);
   }

   public static AppearanceDefinition Coral()
   {
      return RGBColorFromHex(0xFF7F50);
   }

   public static AppearanceDefinition CornflowerBlue()
   {
      return RGBColorFromHex(0x6495ED);
   }

   public static AppearanceDefinition Cornsilk()
   {
      return RGBColorFromHex(0xFFF8DC);
   }

   public static AppearanceDefinition Crimson()
   {
      return RGBColorFromHex(0xDC143C);
   }

   public static AppearanceDefinition Cyan()
   {
      return RGBColorFromHex(0x00FFFF);
   }

   public static AppearanceDefinition DarkBlue()
   {
      return RGBColorFromHex(0x00008B);
   }

   public static AppearanceDefinition DarkCyan()
   {
      return RGBColorFromHex(0x008B8B);
   }

   public static AppearanceDefinition DarkGoldenRod()
   {
      return RGBColorFromHex(0xB8860B);
   }

   public static AppearanceDefinition DarkGray()
   {
      return RGBColorFromHex(0x424242);
   }

   public static AppearanceDefinition DarkGrey()
   {
      return RGBColorFromHex(0x424242);
   }

   public static AppearanceDefinition DarkGreen()
   {
      return RGBColorFromHex(0x006400);
   }

   public static AppearanceDefinition DarkKhaki()
   {
      return RGBColorFromHex(0xBDB76B);
   }

   public static AppearanceDefinition DarkMagenta()
   {
      return RGBColorFromHex(0x8B008B);
   }

   public static AppearanceDefinition DarkOliveGreen()
   {
      return RGBColorFromHex(0x556B2F);
   }

   public static AppearanceDefinition Darkorange()
   {
      return RGBColorFromHex(0xFF8C00);
   }

   public static AppearanceDefinition DarkOrchid()
   {
      return RGBColorFromHex(0x9932CC);
   }

   public static AppearanceDefinition DarkRed()
   {
      return RGBColorFromHex(0x8B0000);
   }

   public static AppearanceDefinition DarkSalmon()
   {
      return RGBColorFromHex(0xE9967A);
   }

   public static AppearanceDefinition DarkSeaGreen()
   {
      return RGBColorFromHex(0x8FBC8F);
   }

   public static AppearanceDefinition DarkSlateBlue()
   {
      return RGBColorFromHex(0x483D8B);
   }

   public static AppearanceDefinition DarkSlateGray()
   {
      return RGBColorFromHex(0x2F4F4F);
   }

   public static AppearanceDefinition DarkSlateGrey()
   {
      return RGBColorFromHex(0x2F4F4F);
   }

   public static AppearanceDefinition DarkTurquoise()
   {
      return RGBColorFromHex(0x00CED1);
   }

   public static AppearanceDefinition DarkViolet()
   {
      return RGBColorFromHex(0x9400D3);
   }

   public static AppearanceDefinition DeepPink()
   {
      return RGBColorFromHex(0xFF1493);
   }

   public static AppearanceDefinition DeepSkyBlue()
   {
      return RGBColorFromHex(0x00BFFF);
   }

   public static AppearanceDefinition DimGray()
   {
      return RGBColorFromHex(0x696969);
   }

   public static AppearanceDefinition DimGrey()
   {
      return RGBColorFromHex(0x696969);
   }

   public static AppearanceDefinition DodgerBlue()
   {
      return RGBColorFromHex(0x1E90FF);
   }

   public static AppearanceDefinition FireBrick()
   {
      return RGBColorFromHex(0xB22222);
   }

   public static AppearanceDefinition FloralWhite()
   {
      return RGBColorFromHex(0xFFFAF0);
   }

   public static AppearanceDefinition ForestGreen()
   {
      return RGBColorFromHex(0x228B22);
   }

   public static AppearanceDefinition Fuchsia()
   {
      return RGBColorFromHex(0xFF00FF);
   }

   public static AppearanceDefinition Gainsboro()
   {
      return RGBColorFromHex(0xDCDCDC);
   }

   public static AppearanceDefinition GhostWhite()
   {
      return RGBColorFromHex(0xF8F8FF);
   }

   public static AppearanceDefinition Gold()
   {
      return RGBColorFromHex(0xFFD700);
   }

   public static AppearanceDefinition GoldenRod()
   {
      return RGBColorFromHex(0xDAA520);
   }

   public static AppearanceDefinition Gray()
   {
      return RGBColorFromHex(0x808080);
   }

   public static AppearanceDefinition Grey()
   {
      return RGBColorFromHex(0x808080);
   }

   public static AppearanceDefinition Green()
   {
      return RGBColorFromHex(0x008000);
   }

   public static AppearanceDefinition GreenYellow()
   {
      return RGBColorFromHex(0xADFF2F);
   }

   public static AppearanceDefinition Glass(double transparency)
   {
      AppearanceDefinition glass = SkyBlue();
      makeTransparent(glass, transparency);//0.5);

      return glass;
   }

   public static AppearanceDefinition Glass()
   {
      return Glass(0.5);
   }

   public static AppearanceDefinition HoneyDew()
   {
      return RGBColorFromHex(0xF0FFF0);
   }

   public static AppearanceDefinition HotPink()
   {
      return RGBColorFromHex(0xFF69B4);
   }

   public static AppearanceDefinition IndianRed()
   {
      return RGBColorFromHex(0xCD5C5C);
   }

   public static AppearanceDefinition Indigo()
   {
      return RGBColorFromHex(0x4B0082);
   }

   public static AppearanceDefinition Ivory()
   {
      return RGBColorFromHex(0xFFFFF0);
   }

   public static AppearanceDefinition Khaki()
   {
      return RGBColorFromHex(0xF0E68C);
   }

   public static AppearanceDefinition Lavender()
   {
      return RGBColorFromHex(0xE6E6FA);
   }

   public static AppearanceDefinition LavenderBlush()
   {
      return RGBColorFromHex(0xFFF0F5);
   }

   public static AppearanceDefinition LawnGreen()
   {
      return RGBColorFromHex(0x7CFC00);
   }

   public static AppearanceDefinition LemonChiffon()
   {
      return RGBColorFromHex(0xFFFACD);
   }

   public static AppearanceDefinition LightBlue()
   {
      return RGBColorFromHex(0xADD8E6);
   }

   public static AppearanceDefinition LightCoral()
   {
      return RGBColorFromHex(0xF08080);
   }

   public static AppearanceDefinition LightCyan()
   {
      return RGBColorFromHex(0xE0FFFF);
   }

   public static AppearanceDefinition LightGoldenRodYellow()
   {
      return RGBColorFromHex(0xFAFAD2);
   }

   public static AppearanceDefinition LightGray()
   {
      return RGBColorFromHex(0xD3D3D3);
   }

   public static AppearanceDefinition LightGrey()
   {
      return RGBColorFromHex(0xD3D3D3);
   }

   public static AppearanceDefinition LightGreen()
   {
      return RGBColorFromHex(0x90EE90);
   }

   public static AppearanceDefinition LightPink()
   {
      return RGBColorFromHex(0xFFB6C1);
   }

   public static AppearanceDefinition LightSalmon()
   {
      return RGBColorFromHex(0xFFA07A);
   }

   public static AppearanceDefinition LightSeaGreen()
   {
      return RGBColorFromHex(0x20B2AA);
   }

   public static AppearanceDefinition LightSkyBlue()
   {
      return RGBColorFromHex(0x87CEFA);
   }

   public static AppearanceDefinition LightSlateGray()
   {
      return RGBColorFromHex(0x778899);
   }

   public static AppearanceDefinition LightSlateGrey()
   {
      return RGBColorFromHex(0x778899);
   }

   public static AppearanceDefinition LightSteelBlue()
   {
      return RGBColorFromHex(0xB0C4DE);
   }

   public static AppearanceDefinition LightYellow()
   {
      return RGBColorFromHex(0xFFFFE0);
   }

   public static AppearanceDefinition Lime()
   {
      return RGBColorFromHex(0x00FF00);
   }

   public static AppearanceDefinition LimeGreen()
   {
      return RGBColorFromHex(0x32CD32);
   }

   public static AppearanceDefinition Linen()
   {
      return RGBColorFromHex(0xFAF0E6);
   }

   public static AppearanceDefinition Magenta()
   {
      return RGBColorFromHex(0xFF00FF);
   }

   public static AppearanceDefinition Maroon()
   {
      return RGBColorFromHex(0x800000);
   }

   public static AppearanceDefinition MediumAquaMarine()
   {
      return RGBColorFromHex(0x66CDAA);
   }

   public static AppearanceDefinition MediumBlue()
   {
      return RGBColorFromHex(0x0000CD);
   }

   public static AppearanceDefinition MediumOrchid()
   {
      return RGBColorFromHex(0xBA55D3);
   }

   public static AppearanceDefinition MediumPurple()
   {
      return RGBColorFromHex(0x9370DB);
   }

   public static AppearanceDefinition MediumSeaGreen()
   {
      return RGBColorFromHex(0x3CB371);
   }

   public static AppearanceDefinition MediumSlateBlue()
   {
      return RGBColorFromHex(0x7B68EE);
   }

   public static AppearanceDefinition MediumSpringGreen()
   {
      return RGBColorFromHex(0x00FA9A);
   }

   public static AppearanceDefinition MediumTurquoise()
   {
      return RGBColorFromHex(0x48D1CC);
   }

   public static AppearanceDefinition MediumVioletRed()
   {
      return RGBColorFromHex(0xC71585);
   }

   public static AppearanceDefinition MidnightBlue()
   {
      return RGBColorFromHex(0x191970);
   }

   public static AppearanceDefinition MintCream()
   {
      return RGBColorFromHex(0xF5FFFA);
   }

   public static AppearanceDefinition MistyRose()
   {
      return RGBColorFromHex(0xFFE4E1);
   }

   public static AppearanceDefinition Moccasin()
   {
      return RGBColorFromHex(0xFFE4B5);
   }

   public static AppearanceDefinition NavajoWhite()
   {
      return RGBColorFromHex(0xFFDEAD);
   }

   public static AppearanceDefinition Navy()
   {
      return RGBColorFromHex(0x000080);
   }

   public static AppearanceDefinition OldLace()
   {
      return RGBColorFromHex(0xFDF5E6);
   }

   public static AppearanceDefinition Olive()
   {
      return RGBColorFromHex(0x808000);
   }

   public static AppearanceDefinition OliveDrab()
   {
      return RGBColorFromHex(0x6B8E23);
   }

   public static AppearanceDefinition Orange()
   {
      return RGBColorFromHex(0xFFA500);
   }

   public static AppearanceDefinition OrangeRed()
   {
      return RGBColorFromHex(0xFF4500);
   }

   public static AppearanceDefinition Orchid()
   {
      return RGBColorFromHex(0xDA70D6);
   }

   public static AppearanceDefinition PaleGoldenRod()
   {
      return RGBColorFromHex(0xEEE8AA);
   }

   public static AppearanceDefinition PaleGreen()
   {
      return RGBColorFromHex(0x98FB98);
   }

   public static AppearanceDefinition PaleTurquoise()
   {
      return RGBColorFromHex(0xAFEEEE);
   }

   public static AppearanceDefinition PaleVioletRed()
   {
      return RGBColorFromHex(0xDB7093);
   }

   public static AppearanceDefinition PapayaWhip()
   {
      return RGBColorFromHex(0xFFEFD5);
   }

   public static AppearanceDefinition PeachPuff()
   {
      return RGBColorFromHex(0xFFDAB9);
   }

   public static AppearanceDefinition Peru()
   {
      return RGBColorFromHex(0xCD853F);
   }

   public static AppearanceDefinition Pink()
   {
      return RGBColorFromHex(0xFFC0CB);
   }

   public static AppearanceDefinition Plum()
   {
      return RGBColorFromHex(0xDDA0DD);
   }

   public static AppearanceDefinition PowderBlue()
   {
      return RGBColorFromHex(0xB0E0E6);
   }

   public static AppearanceDefinition Purple()
   {
      return RGBColorFromHex(0x800080);
   }

   public static AppearanceDefinition Red()
   {
      return RGBColorFromHex(0xFF0000);
   }

   public static AppearanceDefinition RosyBrown()
   {
      return RGBColorFromHex(0xBC8F8F);
   }

   public static AppearanceDefinition RoyalBlue()
   {
      return RGBColorFromHex(0x4169E1);
   }

   public static AppearanceDefinition SaddleBrown()
   {
      return RGBColorFromHex(0x8B4513);
   }

   public static AppearanceDefinition Salmon()
   {
      return RGBColorFromHex(0xFA8072);
   }

   public static AppearanceDefinition SandyBrown()
   {
      return RGBColorFromHex(0xF4A460);
   }

   public static AppearanceDefinition SeaGreen()
   {
      return RGBColorFromHex(0x2E8B57);
   }

   public static AppearanceDefinition SeaShell()
   {
      return RGBColorFromHex(0xFFF5EE);
   }

   public static AppearanceDefinition Sienna()
   {
      return RGBColorFromHex(0xA0522D);
   }

   public static AppearanceDefinition Silver()
   {
      return RGBColorFromHex(0xC0C0C0);
   }

   public static AppearanceDefinition SkyBlue()
   {
      return RGBColorFromHex(0x87CEEB);
   }

   public static AppearanceDefinition SlateBlue()
   {
      return RGBColorFromHex(0x6A5ACD);
   }

   public static AppearanceDefinition SlateGray()
   {
      return RGBColorFromHex(0x708090);
   }

   public static AppearanceDefinition SlateGrey()
   {
      return RGBColorFromHex(0x708090);
   }

   public static AppearanceDefinition Snow()
   {
      return RGBColorFromHex(0xFFFAFA);
   }

   public static AppearanceDefinition SpringGreen()
   {
      return RGBColorFromHex(0x00FF7F);
   }

   public static AppearanceDefinition SteelBlue()
   {
      return RGBColorFromHex(0x4682B4);
   }

   public static AppearanceDefinition Tan()
   {
      return RGBColorFromHex(0xD2B48C);
   }

   public static AppearanceDefinition Teal()
   {
      return RGBColorFromHex(0x008080);
   }

   public static AppearanceDefinition Thistle()
   {
      return RGBColorFromHex(0xD8BFD8);
   }

   public static AppearanceDefinition Tomato()
   {
      return RGBColorFromHex(0xFF6347);
   }

   public static AppearanceDefinition Turquoise()
   {
      return RGBColorFromHex(0x40E0D0);
   }

   public static AppearanceDefinition Violet()
   {
      return RGBColorFromHex(0xEE82EE);
   }

   public static AppearanceDefinition Wheat()
   {
      return RGBColorFromHex(0xF5DEB3);
   }

   public static AppearanceDefinition White()
   {
      return RGBColorFromHex(0xFFFFFF);
   }

   public static AppearanceDefinition WhiteSmoke()
   {
      return RGBColorFromHex(0xF5F5F5);
   }

   public static AppearanceDefinition Yellow()
   {
      return RGBColorFromHex(0xFFFF00);
   }

   public static AppearanceDefinition YellowGreen()
   {
      return RGBColorFromHex(0x9ACD32);
   }




   public static AppearanceDefinition Transparent()
   {
      return new YoAppearanceTransparent();
   }

   public static void makeTransparent(AppearanceDefinition appearance, double f)
   {
      appearance.setTransparency(f);
   }

   public static AppearanceDefinition randomColor(Random random)
   {
      double red = random.nextDouble();
      double green = random.nextDouble();
      double blue = random.nextDouble();

      return YoAppearance.RGBColor(red, green, blue);
   }

   // public static void makeTransparent(YoAppearanceDefinition app, float transparency)
   // {
   // TransparencyAttributes ta = new TransparencyAttributes();
   // ta.setTransparency(transparency);
   // ta.setTransparencyMode(TransparencyAttributes.BLENDED);
   //
   // app.setTransparencyAttributes(ta);
   // }
   //
   // public static void makeLineDrawing(YoAppearanceDefinition app)
   // {
   // PolygonAttributes pa = new PolygonAttributes();
   // pa.setPolygonMode(PolygonAttributes.POLYGON_LINE);
   // app.setPolygonAttributes(pa);
   // }

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
