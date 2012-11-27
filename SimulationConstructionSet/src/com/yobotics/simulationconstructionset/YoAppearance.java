package com.yobotics.simulationconstructionset;

import java.awt.Color;
import java.awt.Component;
import java.net.URL;

import com.yobotics.simulationconstructionset.graphics.YoAppearanceDefinition;
import com.yobotics.simulationconstructionset.graphics.YoAppearanceMaterial;
import com.yobotics.simulationconstructionset.graphics.YoAppearanceRGBColor;
import com.yobotics.simulationconstructionset.graphics.YoAppearanceTexture;
import com.yobotics.simulationconstructionset.graphics.YoAppearanceTransparant;

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

   public static YoAppearanceDefinition YoboticsTexture(Component comp)
   {
      URL fileURL = SimulationConstructionSet.class.getResource("images/yobotics.jpg");
      return Texture(fileURL, comp);
   }

   public static YoAppearanceDefinition Texture(URL fileURL, Component comp)
   {

      return new YoAppearanceTexture(fileURL, comp);

   }

   public static YoAppearanceDefinition EarthTexture(Component comp)
   {

      URL fileURL = SimulationConstructionSet.class.getResource("images/earth.jpg");
      return Texture(fileURL, comp);

   }

   public static YoAppearanceDefinition StoneTexture(Component comp)
   {

      URL fileURL = SimulationConstructionSet.class.getResource("images/stone.jpg");
      return Texture(fileURL, comp);
   }

   public static YoAppearanceDefinition PlaneMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      return mat;
   }

   public static YoAppearanceDefinition AluminumMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(7.5f);
      mat.setAmbientColor(0.17f, 0.5f, 0.7f);
      return mat;
   }

   public static YoAppearanceDefinition BlackMetalMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.5f, 0.5f, 0.5f);
      mat.setDiffuseColor(0.2f, 0.4f, 0.5f);
      mat.setShininess(6.0f);
      mat.setAmbientColor(0.16f, 0.18f, 0.2f);
      return mat;
   }

   public static YoAppearanceDefinition FenceMaterial()
   {
      YoAppearanceMaterial mat = new YoAppearanceMaterial();
      mat.setSpecularColor(0.4f, 0.4f, 0.4f);
      mat.setDiffuseColor(0.95f, 0.95f, 0.95f);
      mat.setShininess(2.0f);
      mat.setAmbientColor(0.45f, 0.45f, 0.45f);
      return mat;
   }

   public static YoAppearanceDefinition RGBColor(double red, double green, double blue)
   {
      return new YoAppearanceRGBColor(red, green, blue);
   }

   public static YoAppearanceDefinition RGBColor(float red, float green, float blue)
   {
      return new YoAppearanceRGBColor(red, green, blue);

   }

   public static YoAppearanceDefinition RGBColorFrom8BitInts(int red, int green, int blue)
   {
      return RGBColor(((red)) / 255.0, ((green)) / 255.0, ((blue)) / 255.0);
   }

   public static YoAppearanceDefinition Color(Color color)
   {
      return RGBColor(color.getRed(), color.getGreen(), color.getBlue());
   }

   public static YoAppearanceDefinition Black()
   {
      return RGBColor(0.0f, 0.0f, 0.0f);
   }

   public static YoAppearanceDefinition White()
   {
      return RGBColor(1.0f, 1.0f, 1.0f);
   }

   public static YoAppearanceDefinition Red()
   {
      return RGBColor(1.0f, 0.0f, 0.0f);
   }

   /**
    * Some colors from: http://cloford.com/resources/colours/500col.htm
    */
   public static YoAppearanceDefinition Pink()
   {
      return RGBColorFrom8BitInts(255, 192, 203);
   }

   public static YoAppearanceDefinition Orange()
   {
      return RGBColorFrom8BitInts(255, 128, 0);
   }

   public static YoAppearanceDefinition Orchid()
   {
      return RGBColorFrom8BitInts(218, 112, 214);
   }

   public static YoAppearanceDefinition DarkRed()
   {
      return RGBColor(0.3f, 0.0f, 0.0f);
   }

   public static YoAppearanceDefinition Blue()
   {
      return RGBColor(0.0f, 0.0f, 1.0f);
   }

   public static YoAppearanceDefinition DarkBlue()
   {
      return RGBColor(0.0f, 0.0f, 0.3f);
   }

   public static YoAppearanceDefinition Brown()
   {
      return RGBColorFrom8BitInts(165, 42, 42);
   }

   public static YoAppearanceDefinition Green()
   {
      return RGBColor(0.0f, 1.0f, 0.0f);
   }

   public static YoAppearanceDefinition DarkGreen()
   {
      return RGBColor(0.0f, 0.3f, 0.0f);
   }

   public static YoAppearanceDefinition Silver()
   {
      return RGBColor(0.75f, 0.75f, 0.75f);
   }

   public static YoAppearanceDefinition Gray()
   {
      return RGBColor(0.50f, 0.50f, 0.50f);
   }

   public static YoAppearanceDefinition Maroon()
   {
      return RGBColor(0.50f, 0.0f, 0.0f);
   }

   public static YoAppearanceDefinition Purple()
   {
      return RGBColor(0.50f, 0.0f, 0.50f);
   }

   public static YoAppearanceDefinition Fuchsia()
   {
      return RGBColor(1.0f, 0.0f, 1.0f);
   }

   public static YoAppearanceDefinition Olive()
   {
      return RGBColor(0.50f, 0.50f, 0.0f);
   }

   public static YoAppearanceDefinition Yellow()
   {
      return RGBColor(1.0f, 1.0f, 0.0f);
   }

   public static YoAppearanceDefinition Navy()
   {
      return RGBColor(0.0f, 0.0f, 0.50f);
   }

   public static YoAppearanceDefinition Teal()
   {
      return RGBColor(0.0f, 0.50f, 0.50f);
   }

   public static YoAppearanceDefinition Aqua()
   {
      return RGBColor(0.0f, 1.0f, 1.0f);
   }

   public static YoAppearanceDefinition Transparent()
   {
      return new YoAppearanceTransparant();
   }

   public static void makeTransparent(YoAppearanceDefinition appearance, float f)
   {
      System.err.println("Transparancy not supported for now");
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
