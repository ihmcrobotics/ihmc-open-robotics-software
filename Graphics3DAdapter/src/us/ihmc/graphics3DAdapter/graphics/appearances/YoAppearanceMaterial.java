package us.ihmc.graphics3DAdapter.graphics.appearances;

import javax.vecmath.Color3f;

public class YoAppearanceMaterial extends YoAppearanceTransparency
{
   
   
   private final Color3f diffuseColor = new Color3f();
   private final Color3f specularColor = new Color3f();
   private float shininess;
   private final Color3f ambientColor = new Color3f();
   

   public void setDiffuseColor(float f, float g, float h)
   {
      diffuseColor.x = f;
      diffuseColor.y = g;
      diffuseColor.z = h;
      
   }

   public void setSpecularColor(float f, float g, float h)
   {
      specularColor.x = f;
      specularColor.y = g;
      specularColor.z = h;
   }

   public void setShininess(float f)
   {
      shininess = f;
   }

   public void setAmbientColor(float f, float g, float h)
   {
      ambientColor.x = f;
      ambientColor.y = g;
      ambientColor.z = h;
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

   public Color3f getColor()
   {
      return diffuseColor;
   }

   
}
