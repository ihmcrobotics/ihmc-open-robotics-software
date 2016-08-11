package us.ihmc.plotting;

public class PlotterTransform
{
   private double metersToPixels;
   private double pixelsToMeters;
   
   public void setMetersToPixels(double metersToPixels)
   {
      this.metersToPixels = metersToPixels;
      updatePixelsToMeters();
   }
   
   public void setPixelsToMeters(double pixelsToMeters)
   {
      this.pixelsToMeters = pixelsToMeters;
      updateMetersToPixels();
   }
   
   private void updatePixelsToMeters()
   {
      pixelsToMeters = 1.0 / metersToPixels;
   }
   
   private void updateMetersToPixels()
   {
      metersToPixels = 1.0 / pixelsToMeters;
   }
   
   public int transformMetersToPixels(double meters)
   {
      return (int) Math.round(meters * metersToPixels);
   }
   
   public double transformPixelsToMeters(int pixels)
   {
      return pixels * pixelsToMeters;
   }

   public double getMetersToPixels()
   {
      return metersToPixels;
   }
   
   public double getPixelsToMeters()
   {
      return pixelsToMeters;
   }
}
