package us.ihmc.plotting;

import javax.vecmath.Point2i;

public class PlotterTransform
{
   private double metersToPixels;
   private double pixelsToMeters;
   private Point2i pixelOriginToMetersOriginInPixels = new Point2i();
   
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
   
   public void setPixelOriginToMetersOriginInPixels(int x, int y)
   {
      pixelOriginToMetersOriginInPixels.set(x, y);
   }
   
   public int transformMetersToPixelsX(double meters)
   {
      return pixelOriginToMetersOriginInPixels.getX() + scaleMetersToPixels(meters);
   }
   
   public double transformPixelsToMetersX(int pixels)
   {
      return scalePixelsToMeters(pixels - pixelOriginToMetersOriginInPixels.getX());
   }
   
   public int transformMetersToPixelsY(double meters)
   {
      return pixelOriginToMetersOriginInPixels.getY() - scaleMetersToPixels(meters);
   }
   
   public double transformPixelsToMetersY(int pixels)
   {
      return scalePixelsToMeters(pixelOriginToMetersOriginInPixels.getY() - pixels);
   }
   
   public int scaleMetersToPixels(double meters)
   {
      return (int) Math.round(meters * metersToPixels);
   }
   
   public double scalePixelsToMeters(int pixels)
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
   
   public Point2i getPixelOriginToMetersOriginInPixels()
   {
      return pixelOriginToMetersOriginInPixels;
   }
}
