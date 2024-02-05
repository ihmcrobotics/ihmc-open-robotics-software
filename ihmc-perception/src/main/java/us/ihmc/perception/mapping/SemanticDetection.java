package us.ihmc.perception.mapping;

public class SemanticDetection
{
   private int centerRowIndex;
   private int centerColumnIndex;
   private int width;
   private int height;
   private float confidence;

   public SemanticDetection(int centerRowIndex, int centerColumnIndex, int width, int height, float confidence)
   {
      this.centerRowIndex = centerRowIndex;
      this.centerColumnIndex = centerColumnIndex;
      this.width = width;
      this.height = height;
      this.confidence = confidence;
   }

   public int getCenterRowIndex()
   {
      return centerRowIndex;
   }

   public int getCenterColumnIndex()
   {
      return centerColumnIndex;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public float getConfidence()
   {
      return confidence;
   }

   public int getTopLeftRow()
   {
      return centerRowIndex - height / 2;
   }

   public int getTopLeftColumn()
   {
      return centerColumnIndex - width / 2;
   }

   public int getBottomRightRow()
   {
      return centerRowIndex + height / 2;
   }

   public int getBottomRightColumn()
   {
      return centerColumnIndex + width / 2;
   }
}
