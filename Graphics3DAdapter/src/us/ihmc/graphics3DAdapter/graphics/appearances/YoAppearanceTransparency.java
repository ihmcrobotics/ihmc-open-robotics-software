package us.ihmc.graphics3DAdapter.graphics.appearances;

public abstract class YoAppearanceTransparency implements YoAppearanceDefinition
{
   private double transparency = 0.0;

   public final void setTransparancy(double transparency)
   {
      this.transparency = transparency;
   }

   public final double getTransparency()
   {
      return transparency;
   }
   
   public void setTransparency(double transparency)
   {
      this.transparency = transparency;
   }
}
