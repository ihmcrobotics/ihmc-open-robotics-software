package us.ihmc.avatar.stereoVision;

public class ImageCropInfo
{
   private double cropWidth;
   private double cropHeight;
   private double xOffset;
   private double yOffset;

   public ImageCropInfo()
   {

   }

   public ImageCropInfo(double cropWidth, double cropHeight, double xOffset, double yOffset)
   {
      this.cropWidth = cropWidth;
      this.cropHeight = cropHeight;
      this.xOffset = xOffset;
      this.yOffset = yOffset;
   }

   public int getCropWidth()
   {
      return (int) cropWidth;
   }

   public int getCropHeight()
   {
      return (int) cropHeight;
   }

   public int getXOffset()
   {
      return (int) xOffset;
   }

   public int getYOffset()
   {
      return (int) yOffset;
   }
}
