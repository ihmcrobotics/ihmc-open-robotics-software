package us.ihmc.perception;

public class PinholeCameraParameters
{
   private final int imageHeight;
   private final int imageWidth;
   private final double fx;
   private final double fy;
   private final double cx;
   private final double cy;

   public PinholeCameraParameters(int imageHeight, int imageWidth, double fx, double fy, double cx, double cy)
   {
      this.imageHeight = imageHeight;
      this.imageWidth = imageWidth;
      this.fx = fx;
      this.fy = fy;
      this.cx = cx;
      this.cy = cy;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public double getFocalLengthX()
   {
      return fx;
   }

   public double getFocalLengthY()
   {
      return fy;
   }

   public double getPrincipalPointX()
   {
      return cx;
   }

   public double getPrincipalPointY()
   {
      return cy;
   }
}
