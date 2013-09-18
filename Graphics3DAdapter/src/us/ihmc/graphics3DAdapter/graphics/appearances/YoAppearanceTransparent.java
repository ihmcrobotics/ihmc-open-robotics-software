package us.ihmc.graphics3DAdapter.graphics.appearances;

import javax.vecmath.Color3f;

import org.apache.commons.lang.NotImplementedException;

public class YoAppearanceTransparent extends YoAppearanceTransparency
{
   public YoAppearanceTransparent()
   {
      setTransparency(1.0);
   }

   @Override
   public Color3f getColor()
   {
      throw new NotImplementedException();
   }

}
