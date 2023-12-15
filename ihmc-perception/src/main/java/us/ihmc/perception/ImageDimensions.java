package us.ihmc.perception;

/**
 * Meant for more safely and conveniently passing around image dimensions
 * in a way not specific to any library so we can make technologically
 * independent abstractions.
 */
public class ImageDimensions
{
   private int imageWidth;
   private int imageHeight;

   public ImageDimensions(int imageWidth, int imageHeight)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public int getColumns()
   {
      return imageWidth;
   }

   public void setColumns(int columns)
   {
      this.imageWidth = imageWidth;
   }

   public int getRows()
   {
      return imageHeight;
   }

   public void setRows(int rows)
   {
      this.imageHeight = imageHeight;
   }
}
