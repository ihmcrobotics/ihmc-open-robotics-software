package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

public class ImageSegmentationParameters
{
   private static final int DEFAULT_PIXEL_SIZE = 30;
   private static final double DEFAULT_PIXEL_RULER = 80.0;
   private static final int DEFAULT_ITERATE = 6;
   private static final int DEFAULT_MIN_ELEMENT_SIZE = 30;

   private int pixelSize;
   private double pixelRuler;
   private int iterate;
   private int minElementSize;

   public ImageSegmentationParameters()
   {
      setDefaultParameters();
   }

   public ImageSegmentationParameters(ImageSegmentationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      pixelSize = DEFAULT_PIXEL_SIZE;
      pixelRuler = DEFAULT_PIXEL_RULER;
      iterate = DEFAULT_ITERATE;
      minElementSize = DEFAULT_MIN_ELEMENT_SIZE;
   }

   public void set(ImageSegmentationParameters other)
   {
      pixelSize = other.pixelSize;
      pixelRuler = other.pixelRuler;
      iterate = other.iterate;
      minElementSize = other.minElementSize;
   }

   public int getPixelSize()
   {
      return pixelSize;
   }

   public double getPixelRuler()
   {
      return pixelRuler;
   }

   public int getIterate()
   {
      return iterate;
   }

   public int getMinElementSize()
   {
      return minElementSize;
   }

   public void setPixelSize(int pixelSize)
   {
      this.pixelSize = pixelSize;
   }

   public void setPixelRuler(double pixelRuler)
   {
      this.pixelRuler = pixelRuler;
   }

   public void setIterate(int iterate)
   {
      this.iterate = iterate;
   }

   public void setMinElementSize(int minElementSize)
   {
      this.minElementSize = minElementSize;
   }
}
