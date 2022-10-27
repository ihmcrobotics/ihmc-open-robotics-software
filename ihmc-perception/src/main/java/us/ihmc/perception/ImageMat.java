package us.ihmc.perception;

public class ImageMat
{
   private byte[] data;
   private int rows;
   private int cols;

   public ImageMat(byte[] data, int rows, int cols)
   {
      this.data = data;
      this.rows = rows;
      this.cols = cols;
   }

   public byte[] getData()
   {
      return data;
   }

   public void setData(byte[] data)
   {
      this.data = data;
   }

   public int getRows()
   {
      return rows;
   }

   public void setRows(int rows)
   {
      this.rows = rows;
   }

   public int getCols()
   {
      return cols;
   }

   public void setCols(int cols)
   {
      this.cols = cols;
   }
}
