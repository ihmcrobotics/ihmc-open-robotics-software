package us.ihmc.graphicsDescription.appearance;

import java.awt.Color;

import javax.vecmath.Color3f;

import org.apache.commons.lang3.NotImplementedException;

public class YoAppearanceMaterial extends YoAppearanceTransparency
{
   private final Color3f diffuseColor = new Color3f();
   private final Color3f specularColor = new Color3f();
   private float shininess;
   private final Color3f ambientColor = new Color3f();

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

   public Color3f getDiffuseColor()
   {
      return diffuseColor;
   }

   public Color3f getSpecularColor()
   {
      return specularColor;
   }

   public float getShininess()
   {
      return shininess;
   }

   public Color3f getAmbientColor()
   {
      return ambientColor;
   }
   
   public void setAmbientColor(Color3f color3f)
   {
      ambientColor.set(color3f);
   }

   public void setDiffuseColor(Color3f color3f)
   {
      diffuseColor.set(color3f);
   }

   public void setSpecularColor(Color3f color3f)
   {
      specularColor.set(color3f);
   }

   @Override
   public Color3f getColor()
   {
      return diffuseColor;
   }

   @Override
   public Color getAwtColor()
   {
      throw new NotImplementedException("getAwtColor() is not implemented");
   }
}
