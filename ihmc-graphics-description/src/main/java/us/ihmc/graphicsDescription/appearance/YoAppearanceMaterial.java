package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.robotics.dataStructures.MutableColor;

public class YoAppearanceMaterial extends YoAppearanceTransparency
{
   private final MutableColor diffuseColor = new MutableColor();
   private final MutableColor specularColor = new MutableColor();
   private float shininess;
   private final MutableColor ambientColor = new MutableColor();

   public void setDiffuseColor(float f, float g, float h)
   {
      diffuseColor.setX(f);
      diffuseColor.setY(g);
      diffuseColor.setZ(h);
   }

   public void setSpecularColor(float f, float g, float h)
   {
      specularColor.setX(f);
      specularColor.setY(g);
      specularColor.setZ(h);
   }

   public void setShininess(float f)
   {
      shininess = f;
   }

   public void setAmbientColor(float f, float g, float h)
   {
      ambientColor.setX(f);
      ambientColor.setY(g);
      ambientColor.setZ(h);
   }

   public MutableColor getDiffuseColor()
   {
      return diffuseColor;
   }

   public MutableColor getSpecularColor()
   {
      return specularColor;
   }

   public float getShininess()
   {
      return shininess;
   }

   public MutableColor getAmbientColor()
   {
      return ambientColor;
   }
   
   public void setAmbientColor(MutableColor color3f)
   {
      ambientColor.set(color3f);
   }

   public void setDiffuseColor(MutableColor color3f)
   {
      diffuseColor.set(color3f);
   }

   public void setSpecularColor(MutableColor color3f)
   {
      specularColor.set(color3f);
   }

   @Override
   public MutableColor getColor()
   {
      return diffuseColor;
   }

   @Override
   public Color getAwtColor()
   {
      throw new NotImplementedException("getAwtColor() is not implemented");
   }
}
