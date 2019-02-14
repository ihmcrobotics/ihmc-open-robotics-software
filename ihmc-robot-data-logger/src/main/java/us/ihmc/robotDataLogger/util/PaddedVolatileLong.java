package us.ihmc.robotDataLogger.util;

import org.openjdk.jol.info.ClassLayout;

/**
 * Padded volatile that is guaranteed to be on its own 64 bit cache line
 * 
 * Cache line are 64 bytes, so 56 bytes before and after the value in question.
 * 
 * @author Jesper Smith
 *
 */
public class PaddedVolatileLong
{
   public volatile long p1, p2, p3, p4, p5, p6, p7 = 8L;
   private volatile long paddedLong;
   public volatile long p8, p9, p10, p11, p12, p13, p14 = 8L;
   

   public PaddedVolatileLong()
   {
      
   }

   public PaddedVolatileLong(long initialValue)
   {
      set(initialValue);
   }

   public long getLong()
   {
      return paddedLong;
   }




   public void set(long value)
   {
      this.paddedLong = value;
   }

   
   
   /**
    * Public function to avoid removal of padding
    * 
    * @return sum of p
    * 
    */
   @Deprecated
   public long avoidPaddingRemoval()
   {
      return p1 + p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9 + p10 + p11 + p12 + p13 + p14;
   }


   public static void main(String[] args)
   {
      System.out.println(ClassLayout.parseClass(PaddedVolatileLong.class).toPrintable());
   }
}
