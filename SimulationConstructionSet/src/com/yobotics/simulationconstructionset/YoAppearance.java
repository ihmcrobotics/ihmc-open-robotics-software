package com.yobotics.simulationconstructionset;

import java.awt.Color;
import java.awt.Component;
import java.net.URL;

import javax.media.j3d.Appearance;
import javax.media.j3d.Material;
import javax.media.j3d.PolygonAttributes;
import javax.media.j3d.Texture;
import javax.media.j3d.TextureAttributes;
import javax.media.j3d.TextureUnitState;
import javax.media.j3d.TransparencyAttributes;
import javax.vecmath.Color3f;

import com.sun.j3d.utils.image.TextureLoader;

//http://www.wdvl.com/Graphics/Colour/  has some of the color names I'm using...

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */


public class YoAppearance
{
   public static Appearance YoboticsTexture(Component comp)
   {
      Appearance ret = new Appearance();

      URL fileURL = SimulationConstructionSet.class.getResource("images/yobotics.jpg");
      TextureLoader loader = new TextureLoader(fileURL, comp);

      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);
      ret.setPolygonAttributes(polyAttributes);

      Texture texture = loader.getTexture();
      texture.setMagFilter(Texture.BASE_LEVEL_LINEAR);
      texture.setBoundaryModeS(Texture.WRAP);
      texture.setBoundaryModeT(Texture.WRAP);
      ret.setTexture(texture);

      TextureAttributes texAttr = new TextureAttributes();
      texAttr.setTextureMode(TextureAttributes.REPLACE);
      ret.setTextureAttributes(texAttr);

      return ret;
   }

   public static Appearance Texture(URL fileURL, Component comp)
   {
      Appearance ret = new Appearance();

      // URL fileURL =  SimulationConstructionSet.class.getResource("images/earth.jpg");
      TextureLoader loader = new TextureLoader(fileURL, comp);

      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);

      // polyAttributes.setCullFace(PolygonAttributes.CULL_BACK);
      ret.setPolygonAttributes(polyAttributes);

      Texture texture = loader.getTexture();
      texture.setMagFilter(Texture.BASE_LEVEL_LINEAR);
      texture.setBoundaryModeS(Texture.WRAP);
      texture.setBoundaryModeT(Texture.WRAP);
      ret.setTexture(texture);

      TextureAttributes texAttr = new TextureAttributes();
      texAttr.setTextureMode(TextureAttributes.REPLACE);
      ret.setTextureAttributes(texAttr);

      return ret;
   }


   public static Appearance EarthTexture(Component comp)
   {
      Appearance ret = new Appearance();

      URL fileURL = SimulationConstructionSet.class.getResource("images/earth.jpg");
      TextureLoader loader = new TextureLoader(fileURL, comp);

      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);

      // polyAttributes.setCullFace(PolygonAttributes.CULL_BACK);
      ret.setPolygonAttributes(polyAttributes);

      Texture texture = loader.getTexture();
      texture.setMagFilter(Texture.BASE_LEVEL_LINEAR);
      texture.setBoundaryModeS(Texture.WRAP);
      texture.setBoundaryModeT(Texture.WRAP);
      ret.setTexture(texture);

      TextureAttributes texAttr = new TextureAttributes();
      texAttr.setTextureMode(TextureAttributes.REPLACE);
      ret.setTextureAttributes(texAttr);

      return ret;
   }

   public static Appearance StoneTexture(Component comp)
   {
      Appearance ret = new Appearance();

      // TextureLoader loader = new TextureLoader("C:/images/stone.jpg", comp);

      URL fileURL = SimulationConstructionSet.class.getResource("images/stone.jpg");

      // URL iconURL =  SimulationConstructionSet.class.getResource("icons/YoGoInPoint24_2.gif");
      TextureLoader loader = new TextureLoader(fileURL, comp);



      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);
      ret.setPolygonAttributes(polyAttributes);

      Texture texture = loader.getTexture();
      texture.setMagFilter(Texture.BASE_LEVEL_LINEAR);
      texture.setBoundaryModeS(Texture.WRAP);
      texture.setBoundaryModeT(Texture.WRAP);
      ret.setTexture(texture);

      TextureAttributes texAttr = new TextureAttributes();
      texAttr.setTextureMode(TextureAttributes.REPLACE);

      ret.setTextureAttributes(texAttr);

      return ret;
   }


   protected static Appearance StoneTextureExperiment(Component comp)
   {
      Appearance ret = new Appearance();

      PolygonAttributes polyAttributes = new PolygonAttributes();
      polyAttributes.setCullFace(PolygonAttributes.CULL_NONE);
      ret.setPolygonAttributes(polyAttributes);

      TextureUnitState textureUnitState[] = new TextureUnitState[2];

      URL fileURL1 = SimulationConstructionSet.class.getResource("images/earth.jpg");
      URL fileURL2 = SimulationConstructionSet.class.getResource("images/stone.jpg");

      TextureLoader loader2 = new TextureLoader(fileURL1, comp);
      TextureLoader loader1 = new TextureLoader(fileURL2, comp);

      Texture texture1 = loader1.getTexture();
      Texture texture2 = loader2.getTexture();

      TextureAttributes texAttr1 = new TextureAttributes();

      // texAttr1.setTextureMode(TextureAttributes.DECAL);

      TextureAttributes texAttr2 = new TextureAttributes();

      // texAttr2.setTextureMode(TextureAttributes.DECAL);

      textureUnitState[0] = new TextureUnitState(texture1, texAttr1, null);

      // textureUnitState[0].setCapability(TextureUnitState.ALLOW_STATE_WRITE);

      textureUnitState[1] = new TextureUnitState(texture2, texAttr2, null);

      // textureUnitState[1].setCapability(TextureUnitState.ALLOW_STATE_WRITE);

      // texture1.setMipMapMode(Texture.MULTI_LEVEL_MIPMAP);
      // texture1.setMagFilter(Texture.BASE_LEVEL_POINT);
      // texture1.setMinFilter(Texture.BASE_LEVEL_POINT);
      // texture1.setBoundaryModeS(Texture.CLAMP);
      // texture1.setBoundaryModeT(Texture.CLAMP);
      // ret.setTexture(texture1);

      // TextureAttributes texAttr = new TextureAttributes();
      // texAttr.setTextureMode(TextureAttributes.REPLACE);
      // ret.setTextureAttributes(texAttr);

      ret.setTextureUnitState(textureUnitState);

      return ret;
   }


   public static Appearance PlaneMaterial()
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      ret.setMaterial(mat);

      return ret;
   }

   public static Appearance AluminumMaterial()
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      ret.setMaterial(mat);

      return ret;
   }

   public static Appearance BlackMetalMaterial()
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(6.0f);
      mat.setAmbientColor(0.16f, 0.18f, 0.2f);
      ret.setMaterial(mat);

      return ret;
   }

   public static Appearance FenceMaterial()
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setSpecularColor(0.4f, 0.4f, 0.4f);
      mat.setDiffuseColor(0.95f, 0.95f, 0.95f);
      mat.setShininess(2.0f);
      mat.setAmbientColor(0.45f, 0.45f, 0.45f);
      ret.setMaterial(mat);

      return ret;
   }

   public static Appearance RGBColor(double red, double green, double blue)
   {
      return RGBColor((float) red, (float) green, (float) blue);
   }

   public static Appearance RGBColor(float red, float green, float blue)
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setAmbientColor(red, green, blue);

      ret.setMaterial(mat);

      return ret;
   }

   public static Appearance RGBColorFrom8BitInts(int red, int green, int blue)
   {
      return RGBColor(((red)) / 255.0, ((green)) / 255.0, ((blue)) / 255.0);
   }

   public static Appearance Color(Color color)
   {
      Appearance ret = new Appearance();
      Material mat = new Material();
      mat.setAmbientColor(new Color3f(color));

      ret.setMaterial(mat);

      return ret;
   }


   public static Appearance Black()
   {
      return RGBColor(0.0f, 0.0f, 0.0f);
   }

   public static Appearance White()
   {
      return RGBColor(1.0f, 1.0f, 1.0f);
   }

   public static Appearance Red()
   {
      return RGBColor(1.0f, 0.0f, 0.0f);
   }

   /**
    * Some colors from: http://cloford.com/resources/colours/500col.htm
    */
   public static Appearance Pink()
   {
      return RGBColorFrom8BitInts(255, 192, 203);
   }


   public static Appearance Orange()
   {
      return RGBColorFrom8BitInts(255, 128, 0);
   }

   public static Appearance Orchid()
   {
      return RGBColorFrom8BitInts(218, 112, 214);
   }


   public static Appearance DarkRed()
   {
      return RGBColor(0.3f, 0.0f, 0.0f);
   }

   public static Appearance Blue()
   {
      return RGBColor(0.0f, 0.0f, 1.0f);
   }

   public static Appearance DarkBlue()
   {
      return RGBColor(0.0f, 0.0f, 0.3f);
   }

   public static Appearance Green()
   {
      return RGBColor(0.0f, 1.0f, 0.0f);
   }

   public static Appearance DarkGreen()
   {
      return RGBColor(0.0f, 0.3f, 0.0f);
   }

   public static Appearance Silver()
   {
      return RGBColor(0.75f, 0.75f, 0.75f);
   }

   public static Appearance Gray()
   {
      return RGBColor(0.50f, 0.50f, 0.50f);
   }

   public static Appearance Maroon()
   {
      return RGBColor(0.50f, 0.0f, 0.0f);
   }

   public static Appearance Purple()
   {
      return RGBColor(0.50f, 0.0f, 0.50f);
   }

   public static Appearance Fuchsia()
   {
      return RGBColor(1.0f, 0.0f, 1.0f);
   }

   public static Appearance Olive()
   {
      return RGBColor(0.50f, 0.50f, 0.0f);
   }

   public static Appearance Yellow()
   {
      return RGBColor(1.0f, 1.0f, 0.0f);
   }

   public static Appearance Navy()
   {
      return RGBColor(0.0f, 0.0f, 0.50f);
   }

   public static Appearance Teal()
   {
      return RGBColor(0.0f, 0.50f, 0.50f);
   }

   public static Appearance Aqua()
   {
      return RGBColor(0.0f, 1.0f, 1.0f);
   }

   public static void makeTransparent(Appearance app, float transparency)
   {
      TransparencyAttributes ta = new TransparencyAttributes();
      ta.setTransparency(transparency);
      ta.setTransparencyMode(TransparencyAttributes.BLENDED);

      app.setTransparencyAttributes(ta);
   }

   public static void makeLineDrawing(Appearance app)
   {
      PolygonAttributes pa = new PolygonAttributes();
      pa.setPolygonMode(PolygonAttributes.POLYGON_LINE);
      app.setPolygonAttributes(pa);
   }


   /*
    * public static Appearance Transparent()
    * {
    * Appearance ret = new Appearance();
    * Material mat = new Material();
    * mat.setAmbientColor(0.0f,0.0f,0.0f);
    *
    * TransparencyAttributes ta = new TransparencyAttributes();
    * ta.setTransparency(0.5f);
    * ta.setTransparencyMode(TransparencyAttributes.BLENDED);
    *
    * ret.setTransparencyAttributes(ta);
    * ret.setMaterial(mat);
    * return ret;
    * }
    */

   // public YoAppearance()
   // {
   // }
}
